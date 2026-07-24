"""Self-check for the replay harness. python tools/test_replay.py"""
import replay_intent as R


def fsm(act, raw=None):
    return R.run_fsm(act, 100.0, 50.0, 300.0, raw=raw or [2000] * len(act))


def main():
    n = R.SAMPLE_HZ

    # rest -> no close
    _, st = fsm([0.0] * (2 * n))
    assert set(st) == {R.ST_OPEN}, "quiet signal must not trigger"

    # 2 s rest, 3 s burst -> closes, and the grip scales with activity
    act = [0.0] * (2 * n) + [200.0] * (3 * n)
    intents, st = fsm(act)
    assert R.ST_CLOSING in st, "a clear burst must close"
    assert max(intents) > R.CLOSE_MIN, "grip must scale above the floor"
    on = next(i for i, s in enumerate(st) if s == R.ST_CLOSING)
    assert 2 * n <= on * R.TICK_DIV <= 2 * n + n // 2, "onset within ~0.5 s of truth"

    # burst while the front-end is browned out -> stays OPEN (safety)
    _, st = fsm(act, raw=[0] * len(act))
    assert set(st) == {R.ST_OPEN}, "dead chB must never drive the hand"

    # per-sample thresholds are honoured (rising threshold outruns the signal)
    ramp = [float(i) / n * 200 for i in range(5 * n)]
    _, st = R.run_fsm(ramp, [9e9] * len(ramp), [0.0] * len(ramp),
                      [9e9] * len(ramp), raw=[2000] * len(ramp))
    assert set(st) == {R.ST_OPEN}, "unreachable moving threshold must never fire"

    # every plugin in tools/algos/ loads and is strictly causal in shape
    R.load_plugins()
    print("loaded:", ", ".join(R.ALGOS))
    print("ok")


if __name__ == "__main__":
    main()
