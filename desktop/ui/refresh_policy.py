"""Pure refresh cadence, change detection, backpressure and reconnect rules.

职责：提供刷新分频、变化检测、慢 tick 判定和断线自愈阈值的纯函数。
线程：无状态，由 NiceGUI 页面事件循环调用。
输入：当前叶子页面、单调时钟、已显示的值、刷新/重载计数。
输出：是否刷新、是否变化、是否慢 tick、是否允许自动重载。
状态所有权：不持有状态；时间戳和显示缓存由调用方保存。
禁止依赖：禁止导入 NiceGUI、串口、协议和控制器。
"""
from __future__ import annotations

FAST_INTERVAL_S = 0.1
HEAVY_INTERVAL_S = 0.5
PLOTS_INTERVAL_S = 1.0

# Leaves whose refresher builds charts from curve/component history.
PLOT_LEAVES = ('Curves',)
# Only this leaf consumes snapshot['curves'] / snapshot['control_components'].
CURVE_LEAVES = ('Curves',)

# A tick blocking the event loop longer than this is logged to System Log.
SLOW_TICK_LIMIT_MS = 50.0

# Hidden heartbeat label class shared by browser_view and the injected JS.
HEARTBEAT_LABEL_CLASS = 'ui-heartbeat'
HEARTBEAT_INTERVAL_S = 0.5
HEARTBEAT_POLL_MS = 500
HEARTBEAT_STALE_S = 3.0
AUTO_RELOAD_DELAY_MS = 1000
MAX_AUTO_RELOADS = 2


def active_leaf(group, child, groups, children):
    """Resolve the leaf page name from the selected main tab and sub-tab."""
    if group not in groups:
        group = groups[0]
    options = children.get(group)
    if not options:
        return group
    return child if child in options else options[0]


def select_intervals(leaf):
    """Return (fast, heavy, plots) timer intervals in seconds.

    The plots interval is None when the active leaf does not render charts, so a
    caller can keep one timer and decide per tick instead of rescheduling.
    """
    return (FAST_INTERVAL_S, HEAVY_INTERVAL_S,
            PLOTS_INTERVAL_S if leaf in PLOT_LEAVES else None)


def needs_curves(leaf):
    """Only the Curves page reads curve and control-component history."""
    return leaf in CURVE_LEAVES


def should_refresh_plots(leaf, last_leaf, last_ts, now, interval_s=PLOTS_INTERVAL_S):
    """Charts refresh at most once per interval and only on chart leaves."""
    if leaf not in PLOT_LEAVES:
        return False
    if last_leaf != leaf:
        return True
    if last_ts is None:
        return True
    return now - last_ts >= interval_s


def text_changed(previous, new):
    """Report whether a displayed value must be written again."""
    return previous != new


def changed(cache, key, value):
    """Record value under key and report whether the displayed value changed."""
    if key in cache and not text_changed(cache[key], value):
        return False
    cache[key] = value
    return True


def backpressure_ok(refreshing):
    """Skip a tick while the previous tick of the same kind is still running."""
    return not refreshing


def slow_tick(elapsed_ms, limit_ms=SLOW_TICK_LIMIT_MS):
    """Report ticks that block the page event loop longer than the limit."""
    return elapsed_ms > limit_ms


def heartbeat_due(last_ts, now, interval_s=HEARTBEAT_INTERVAL_S):
    """Throttle hidden heartbeat writes while keeping the stale window short."""
    return last_ts is None or now - last_ts >= interval_s


def heartbeat_stale(elapsed_s, limit_s=HEARTBEAT_STALE_S):
    """A visible page whose heartbeat has not changed for too long is disconnected."""
    return elapsed_s > limit_s


def reload_budget_exhausted(reload_count, max_reloads=MAX_AUTO_RELOADS):
    """Stop the automatic reload loop once the consecutive budget is used up."""
    return reload_count >= max_reloads


def should_auto_reload(visible, elapsed_s, reload_count, max_reloads=MAX_AUTO_RELOADS):
    """Only a visible, stale page with reload budget left may reload itself."""
    if not visible:
        return False
    if not heartbeat_stale(elapsed_s):
        return False
    return not reload_budget_exhausted(reload_count, max_reloads)


ALERT_MAX_AGE_S = 15 * 60


def calibration_alert_visible(operation, active_leaf, now, max_age_s=ALERT_MAX_AGE_S):
    """Only the page that started a calibration shows its alert, once."""
    if not isinstance(operation, dict):
        return False
    alert = operation.get('calibration_alert')
    if not alert or operation.get('alert_acknowledged'):
        return False
    origin = operation.get('origin')
    if not origin or origin != active_leaf:
        return False
    finished_at = operation.get('finished_at')
    if finished_at and now - float(finished_at) > max_age_s:
        return False
    return True
