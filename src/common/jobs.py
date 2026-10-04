"""Ship-side confirmation of engagements ("jobs").  No zenoh dependency, so it is unit-tested.

A drone that wins its auction is only tentatively engaged: it publishes an award and waits for
the ship's ACK.  The ship collects the awards for a threat over a short window, then confirms the
best bids up to the number of drones still needed, giving each a slot around the engagement
point; the rest are rejected.  Only the ship hears every drone, so its decision holds even when
two drones cannot hear each other and both believe they won.
"""

from typing import Dict, List, Tuple


def arbitrate(pending: Dict[str, float], confirmed: Dict[str, int], need: int,
              n_slots: int) -> Tuple[Dict[str, int], List[str]]:
    """Choose which pending awards to confirm.

    pending: drone -> bid cost of awards awaiting a decision; confirmed: drone -> slot already
    held; need: drones still wanted; n_slots: slots around the engagement point (the level).
    Lowest cost wins, ties broken by drone id (the auction's order).  Free slots then go to the
    accepted drones in drone-id order: the same convention drones use for their tentative slot
    (rank among the winners by id), so confirmation does not move a drone that is already flying.
    Assigning by bid order made drones swap slots across the formation after confirmation and
    miss.  Returns (accepted drone -> slot, rejected drones).
    """
    taken = set(confirmed.values())
    free = [s for s in range(n_slots) if s not in taken]
    winners, rejected = [], []
    for agent, _ in sorted(pending.items(), key=lambda kv: (float(kv[1]), str(kv[0]))):
        if agent in confirmed:
            continue
        if len(winners) < min(need, len(free)):
            winners.append(agent)
        else:
            rejected.append(agent)
    accepted = dict(zip(sorted(winners), free))
    return accepted, rejected


def heartbeat_answer(status: str, holder: bool, pending: bool, rejected: bool, hb_confirmed: bool) -> str:
    """What the ship sends a drone whose heartbeat names this job (ACKs, awards and job updates can
    all be lost; the heartbeat says what the drone believes, and the answer fixes a disagreement).

    status: the threat's status ("approved" while the job is open); holder: we confirmed the drone;
    pending: its award awaits arbitration; rejected: we refused this award (drone, order) before;
    hb_confirmed: the drone believes it is confirmed.  Returns:
      "job"     a job update: the job is closed, or the drone was dropped (holders without it);
      "ack"     the ACK again (a holder that has not seen it);
      "nack"    the NACK again;
      "pending" take the heartbeat as the award (its award never arrived) into arbitration;
      "none"    views agree, or arbitration is under way.
    """
    if status != "approved":
        return "job"
    if holder:
        return "none" if hb_confirmed else "ack"
    if hb_confirmed:
        return "job"
    if pending:
        return "none"
    return "nack" if rejected else "pending"
