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
