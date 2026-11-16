// Bus-mode failure fallback: when a bus's channel-based escape fails under
// the registered knobs (fine pitch leaves pad channels too tight for a via,
// while the four-pad interstice still fits), the bus-atomic layer decision
// remains valid - only the channel via geometry is not. Rescue such buses
// through the interstitial dogbone site class: one dogbone via per
// connection at a matched interstice, then a target-layer continuation from
// the via out to the shared boundary so the plan satisfies the same
// breakout-connectivity and copper DRC proofs as every other strategy.
// Buses are matched sequentially, each bus's vias and stubs blocking the
// next bus's matcher, so two buses on one component never interleave their
// interstice fields. Interstitial vias can share a column (stacked
// interstices), so straight continuations can collinearly overlap; exit
// lanes are packed per target layer and exit edge, ordered toward the edge,
// with the jog free to drop half a pitch off the via row - same-row jogs
// are walled by row-mate vias in dense stacked fields, and the inter-row
// channel unwalls them. Buses whose assigned layer equals their source
// layer are skipped - their failure is not a via-geometry failure.
import type { SimpleRouteJson } from "@tscircuit/capacity-autorouter"
import { getAllRoutedTraceCopper } from "./get-routed-trace-copper"
import { getCopperLayerNames } from "./layer-names"
import { matchComponentDogboneViaSites } from "./match-component-dogbone-via-sites"
import { fanoutPlansAreClear } from "./route-bus"
import { buildViaMinimalWindingPlan } from "./route-via-minimal-winding"
import type { FanoutRoutePlan, PreparedBus, PreparedConnection } from "./types"
import { validateRoutedCopperDrc } from "./validate-routed-copper-drc"

type Pt = { x: number; y: number }
type Leg = { start: Pt; end: Pt }

const pointSegDist = (p: Pt, a: Pt, b: Pt): number => {
  const vx = b.x - a.x
  const vy = b.y - a.y
  const l2 = vx * vx + vy * vy
  const t = l2 === 0 ? 0 : Math.max(0, Math.min(1, ((p.x - a.x) * vx + (p.y - a.y) * vy) / l2))
  return Math.hypot(p.x - (a.x + t * vx), p.y - (a.y + t * vy))
}

const segmentsProperlyIntersect = (s1: Leg, s2: Leg): boolean => {
  const rx = s1.end.x - s1.start.x
  const ry = s1.end.y - s1.start.y
  const sx = s2.end.x - s2.start.x
  const sy = s2.end.y - s2.start.y
  const denom = rx * sy - ry * sx
  if (denom === 0) return false
  const qpx = s2.start.x - s1.start.x
  const qpy = s2.start.y - s1.start.y
  const t = (qpx * sy - qpy * sx) / denom
  const u = (qpx * ry - qpy * rx) / denom
  return t >= 0 && t <= 1 && u >= 0 && u <= 1
}

const segSegDist = (s1: Leg, s2: Leg, width: number): number => {
  if (segmentsProperlyIntersect(s1, s2)) return 0 - width
  const d = Math.min(
    pointSegDist(s1.start, s2.start, s2.end),
    pointSegDist(s1.end, s2.start, s2.end),
    pointSegDist(s2.start, s1.start, s1.end),
    pointSegDist(s2.end, s1.start, s1.end),
  )
  return d - width
}

const cardinalExit = (bus: PreparedBus): "bottom" | "top" | "left" | "right" | null => {
  const edge = bus.exitEdge ?? bus.preferredExit
  if (edge === "bottom" || edge === "top" || edge === "left" || edge === "right") return edge
  return null
}

export function fallbackFailedBusDogbones(params: {
  inputSrj: SimpleRouteJson
  plans: readonly FanoutRoutePlan[]
  failedBuses: readonly PreparedBus[]
  busLayerAssignments: Readonly<Record<string, string>>
  clearance: number
  traceWidth: number
  viaDiameter: number
  viaHoleDiameter: number
  allowBlindAndBuriedVias: boolean
}): { plans: FanoutRoutePlan[]; rescuedBusIds: string[] } | null {
  const { inputSrj, plans, failedBuses, busLayerAssignments, clearance } = params
  const layerNames = getCopperLayerNames(inputSrj.layerCount)
  const eligible = failedBuses.filter((bus) => {
    const targetLayer = busLayerAssignments[bus.busId]
    if (!targetLayer) return false
    if (bus.termination.type !== "boundary") return false
    if (bus.connections.length === 0) return false
    return bus.connections.every(
      (connection) => connection.sourceLayer !== targetLayer,
    )
  })
  if (!eligible.length) return null
  const supplied = getAllRoutedTraceCopper(inputSrj, false)
  const bounds = eligible[0]!.sharedBoundary

  const baseBlockingSegments = [
    ...plans.flatMap((plan) =>
      [...plan.segments, ...(plan.planeEndpointSegments ?? [])].map(
        (segment) => ({ connectionIndex: plan.connectionIndex, segment }),
      ),
    ),
    ...supplied.flatMap((copper) =>
      copper.segments.map((segment) => ({ connectionIndex: -1, segment })),
    ),
  ]
  const baseBlockingVias = [
    ...plans.flatMap((plan) =>
      [plan.via, ...(plan.additionalVias ?? []), plan.planeEndpointVia]
        .filter((v) => v !== undefined)
        .map((v) => ({ connectionIndex: plan.connectionIndex, ...v })),
    ),
    ...supplied.flatMap((copper) =>
      copper.vias.map((v) => ({ connectionIndex: -1, ...v })),
    ),
  ]

  // Phase 1 - one joint match across all eligible buses: adjacent byte
  // fields share interstitial space, so the sites must be negotiated
  // together, not bus by bus.
  type Pending = {
    bus: PreparedBus
    viaPoint: Pt
    connection: PreparedConnection
    connectionIndex: number
    targetLayer: string
  }
  const pending: Pending[] = []
  const rescuedBusIds: string[] = []
  const sites = matchComponentDogboneViaSites(eligible, {
    traceWidth: params.traceWidth,
    clearance,
    viaDiameter: params.viaDiameter,
    viaHoleDiameter: params.viaHoleDiameter,
    maximumSearchStates: 10_000,
    additionalObstacles: inputSrj.obstacles,
    blockingSegments: baseBlockingSegments,
    blockingVias: baseBlockingVias,
  })
  if (!sites) {
    return null
  }
  for (const bus of eligible) {
    const targetLayer = busLayerAssignments[bus.busId]!
    let complete = true
    for (const connection of bus.connections) {
      const point = sites.get(connection.connectionIndex)
      if (!point) {
        complete = false
        break
      }
      const dx = point.x - connection.sourcePoint.x
      const dy = point.y - connection.sourcePoint.y
      if (
        Math.min(
          Math.abs(dx),
          Math.abs(dy),
          Math.abs(Math.abs(dx) - Math.abs(dy)),
        ) > 1e-7
      ) {
        complete = false
        break
      }
    }
    if (!complete) continue
    for (const connection of bus.connections) {
      const point = sites.get(connection.connectionIndex)!
      pending.push({ bus, viaPoint: point, connection, connectionIndex: connection.connectionIndex, targetLayer })
    }
    rescuedBusIds.push(bus.busId)
  }
  if (!pending.length) {
    return null
  }
  const laneStep = params.traceWidth + 2 * clearance
  const allViaPoints = pending.map((item) => item.viaPoint)
  const committed: { leg: Leg; layer: string }[] = []
  const legClear = (legs: readonly Leg[], layer: string, ownVia: Pt, extra: readonly { leg: Leg; layer: string }[]): boolean => {
    for (const leg of legs) {
      for (const { leg: other, layer: otherLayer } of [...committed, ...extra]) {
        if (otherLayer !== layer) continue
        if (segSegDist(leg, other, params.traceWidth) < clearance) return false
      }
      for (const vp of allViaPoints) {
        if (vp === ownVia) continue
        if (pointSegDist(vp, leg.start, leg.end) < params.viaDiameter / 2 + params.traceWidth / 2 + clearance) {
          return false
        }
      }
    }
    return true
  }
  type Placed = { targetPoints: Pt[]; exitPoint: Pt }
  const exitLegs = new Map<number, Placed>()
  const groups = new Map<
    string,
    { edge: "bottom" | "top" | "left" | "right"; items: Pending[] }
  >()
  for (const item of pending) {
    const edge = cardinalExit(item.bus)
    if (!edge) continue
    const key = `${item.targetLayer}|${edge}`
    const group = groups.get(key) ?? { edge, items: [] }
    group.items.push(item)
    groups.set(key, group)
  }
  for (const group of groups.values()) {
    const vertical = group.edge === "bottom" || group.edge === "top"
    const towardEdge = group.edge === "bottom" || group.edge === "left" ? 1 : -1
    const byDistance = (sign: number) =>
      [...group.items].sort((a, b) => {
        const pa = vertical ? a.viaPoint.y : a.viaPoint.x
        const pb = vertical ? b.viaPoint.y : b.viaPoint.x
        return (pa - pb) * towardEdge * sign || a.viaPoint.x - b.viaPoint.x
      })
    const viaPoints = group.items.map((item) => item.viaPoint)
    const coords = [...new Set(viaPoints.map((vp) => (vertical ? vp.x : vp.y)))].sort((a, b) => a - b)
    const midpoints: number[] = []
    for (let i = 1; i < coords.length; i++) midpoints.push((coords[i - 1]! + coords[i]!) / 2)
    const spanMin = vertical ? bounds.minX : bounds.minY
    const spanMax = vertical ? bounds.maxX : bounds.maxY
    const edgeRuns: number[] = []
    for (let m = 1; m <= 4; m++) {
      edgeRuns.push(coords[0]! - m * laneStep, coords.at(-1)! + m * laneStep)
    }
    // Greedy packing starves constrained rows (a via above a sibling via can
    // never drop straight and shares its two nearest lanes with the whole
    // column band); search the (lane, jog-row) assignment with a bounded DFS,
    // most-constrained item first.
    const itemOptions = new Map<number, { lane: number; offset: number }[]>()
    for (const item of group.items) {
      const p = item.viaPoint
      const perp = vertical ? p.x : p.y
      const jogLanes = [...new Set<number>([perp, ...midpoints, ...edgeRuns])]
        .filter((lane) => lane >= spanMin && lane <= spanMax)
        .sort((a, b) => Math.abs(a - perp) - Math.abs(b - perp))
      const jogOffsets = [0]
      for (let m = 1; m <= 3; m++) jogOffsets.push(m * laneStep, -m * laneStep)
      const options: { lane: number; offset: number }[] = []
      for (const lane of jogLanes) for (const offset of jogOffsets) options.push({ lane, offset })
      itemOptions.set(item.connectionIndex, options)
    }
    const legsFor = (item: Pending, lane: number, offset: number): Leg[] => {
      const p = item.viaPoint
      const jogAlong = vertical ? { x: p.x, y: p.y + offset } : { x: p.x + offset, y: p.y }
      const jogB = vertical ? { x: lane, y: jogAlong.y } : { x: jogAlong.x, y: lane }
      const edgeEnd = vertical
        ? { x: lane, y: group.edge === "bottom" ? bounds.minY : bounds.maxY }
        : { x: group.edge === "left" ? bounds.minX : bounds.maxX, y: lane }
      return [
        { start: p, end: jogAlong },
        { start: jogAlong, end: jogB },
        { start: jogB, end: edgeEnd },
      ].filter(
        (leg) => Math.abs(leg.start.x - leg.end.x) > 1e-9 || Math.abs(leg.start.y - leg.end.y) > 1e-9,
      )
    }
    // Search in nearest-first item order with nearest-lane options first, so
    // the search follows the proven greedy path and only backtracks where the
    // greedy starves a constrained row.
    const remaining = [...group.items].sort((a, b) => {
      const pa = vertical ? a.viaPoint.y : a.viaPoint.x
      const pb = vertical ? b.viaPoint.y : b.viaPoint.x
      return (pa - pb) * towardEdge || a.viaPoint.x - b.viaPoint.x
    })
    const searchDeadline = performance.now() + 5_000
    const searchState = new Map<number, { lane: number; offset: number }>()
    const localCommitted: { leg: Leg; layer: string }[] = []
    const dfs = (index: number): boolean => {
      if (index === remaining.length) return true
      if (performance.now() > searchDeadline) return false
      const item = remaining[index]!
      const p = item.viaPoint
      for (const { lane, offset } of itemOptions.get(item.connectionIndex)!) {
        const legs = legsFor(item, lane, offset)
        if (!legClear(legs, item.targetLayer, p, localCommitted)) continue
        for (const leg of legs) localCommitted.push({ leg, layer: item.targetLayer })
        searchState.set(item.connectionIndex, { lane, offset })
        if (dfs(index + 1)) return true
        searchState.delete(item.connectionIndex)
        for (let k = 0; k < legs.length; k++) localCommitted.pop()
      }
      return false
    }
    const solved = dfs(0)
    if (!solved) {
      continue
    }
    committed.push(...localCommitted)
    for (const item of group.items) {
      const { lane, offset } = searchState.get(item.connectionIndex)!
      const legs = legsFor(item, lane, offset)
      const targetPoints: Pt[] = [item.viaPoint]
      for (const leg of legs) targetPoints.push(leg.end)
      exitLegs.set(item.connectionIndex, { targetPoints, exitPoint: targetPoints.at(-1)! })
    }
  }

  // Phase 3 - plan construction; a bus whose lanes did not pack is dropped.
  const added: FanoutRoutePlan[] = []
  const rescued = rescuedBusIds.filter((busId) => {
    const bus = eligible.find((candidate) => candidate.busId === busId)!
    if (!bus.connections.every((connection) => exitLegs.has(connection.connectionIndex))) {
      return false
    }
    for (const connection of bus.connections) {
      const placed = exitLegs.get(connection.connectionIndex)!
      added.push(
        buildViaMinimalWindingPlan({
          bus,
          terminal: { connection, viaPoint: placed.targetPoints[0]!, exitPoint: placed.exitPoint },
          targetLayer: busLayerAssignments[bus.busId]!,
          targetLayerPoints: placed.targetPoints,
          layerNames,
          traceWidth: params.traceWidth,
          viaDiameter: params.viaDiameter,
          viaHoleDiameter: params.viaHoleDiameter,
          allowBlindAndBuriedVias: params.allowBlindAndBuriedVias,
        }),
      )
    }
    return true
  })
  if (!added.length) {
    return null
  }
  const candidate = [...plans, ...added]
  const plansClear = fanoutPlansAreClear({
    plans: candidate,
    srj: inputSrj,
    sharedBoundary: bounds,
    clearance,
    allowBlindAndBuriedVias: params.allowBlindAndBuriedVias,
    allowSameNetMerges: false,
  })
  if (!plansClear) {
    return null
  }
  const validation = validateRoutedCopperDrc({
    inputSrj,
    routedSrj: {
      ...inputSrj,
      traces: [
        ...(inputSrj.traces ?? []),
        ...candidate.flatMap((plan) => [
          plan.trace,
          ...(plan.planeEndpointTrace ? [plan.planeEndpointTrace] : []),
        ]),
      ],
    },
    clearance,
    allowBlindAndBuriedVias: params.allowBlindAndBuriedVias,
  })
  return validation.valid ? { plans: candidate, rescuedBusIds: rescued } : null
}
