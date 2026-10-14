"""Predictive inventory intelligence.

Turns the rover's own inspection history into a consumption rate, a
days-to-stockout estimate and a restocking priority.

Why the rate is measured the way it is
--------------------------------------
A naive linear fit over stock level looks reasonable until someone refills a
shelf. The refill is a step increase, the regression reads it as "stock is
growing", and the forecast reports that nothing ever runs out — exactly when
the dashboard most needs to be right.

So consumption is measured from *depletions only*: sum every drop between
consecutive readings and divide by the elapsed time. Refills are ignored,
because a refill is supply, not demand. Readings are first averaged into
hourly buckets, so a burst of scans in one patrol cannot outvote a week of
history.

Both steps are easy to defend out loud: we add up how much stock left the
shelf, and divide by how long it took.
"""

from __future__ import annotations

from datetime import timedelta

from .db import repo

PRIORITY_ORDER = {"Restock now": 0, "Critical": 1, "High": 2, "Watch": 3, "Stable": 4}

CRITICAL_DAYS = 1.5
HIGH_DAYS = 3.5
WATCH_DAYS = 7.0

MIN_BUCKETS = 4            # fewer readings than this is not a trend
MIN_SPAN_DAYS = 0.25       # too little elapsed time to talk about a daily rate
RECENT_WINDOW_DAYS = 2.0   # weight the last two days against the long run
RECENT_WEIGHT = 0.5


def _bucket_hourly(history: list[tuple]) -> list[tuple]:
    """Average readings that share an hour, oldest first."""
    buckets: dict = {}
    for ts, qty in history:
        key = ts.replace(minute=0, second=0, microsecond=0)
        acc = buckets.setdefault(key, [0.0, 0])
        acc[0] += float(qty)
        acc[1] += 1
    return [(k, buckets[k][0] / buckets[k][1]) for k in sorted(buckets)]


def _depletion_rate(series: list[tuple]) -> float:
    """Units consumed per day, ignoring refills."""
    if len(series) < 2:
        return 0.0
    span_days = (series[-1][0] - series[0][0]).total_seconds() / 86400.0
    if span_days <= 0:
        return 0.0
    drops = sum(max(0.0, series[i - 1][1] - series[i][1])
                for i in range(1, len(series)))
    return drops / span_days


def _restock_count(series: list[tuple], threshold: float = 1.5) -> int:
    return sum(1 for i in range(1, len(series))
               if series[i][1] - series[i - 1][1] > threshold)


def forecast_product(product: str, expected_qty: int, shelf_id: str) -> dict | None:
    history = repo.product_history(product, limit=400)
    if len(history) < MIN_BUCKETS:
        return None

    series = _bucket_hourly(history)
    if len(series) < MIN_BUCKETS:
        return None

    span = (series[-1][0] - series[0][0]).total_seconds() / 86400.0
    if span < MIN_SPAN_DAYS:
        return None

    rate = _depletion_rate(series)

    # Blend in the last couple of days so a change in demand shows up quickly.
    cutoff = series[-1][0] - timedelta(days=RECENT_WINDOW_DAYS)
    recent = [p for p in series if p[0] >= cutoff]
    if len(recent) >= MIN_BUCKETS:
        recent_rate = _depletion_rate(recent)
        rate = (1 - RECENT_WEIGHT) * rate + RECENT_WEIGHT * recent_rate

    current = float(history[-1][1])              # latest actual count
    days_left = round(current / rate, 1) if rate > 0.05 else None

    if current <= 0:
        priority = "Restock now"
    elif days_left is None:
        priority = "Stable"
    elif days_left <= CRITICAL_DAYS:
        priority = "Critical"
    elif days_left <= HIGH_DAYS:
        priority = "High"
    elif days_left <= WATCH_DAYS:
        priority = "Watch"
    else:
        priority = "Stable"

    return {
        "product": product,
        "shelf_id": shelf_id,
        "current_qty": int(round(current)),
        "expected_qty": expected_qty,
        "consumption_per_day": round(rate, 2),
        "days_to_stockout": days_left,
        "priority": priority,
        "samples": len(history),
        "span_days": round(span, 1),
        "restocks_observed": _restock_count(series),
        "trend": [int(round(v)) for _, v in series[-16:]],
    }


def all_forecasts() -> list[dict]:
    out = []
    for shelf_id, products in repo.planogram().items():
        for product, expected in products.items():
            f = forecast_product(product, expected, shelf_id)
            if f:
                out.append(f)
    out.sort(key=lambda f: (PRIORITY_ORDER[f["priority"]],
                            f["days_to_stockout"] if f["days_to_stockout"] is not None
                            else 1e9))
    return out
