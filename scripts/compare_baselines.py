"""S1 repeatability check: two baseline JSONs must agree per case within
max(0.15 m, 30%). Exit 0 = repeatable."""
import json
import sys


def main(a_path, b_path):
    a = {r["case"]: r for r in json.load(open(a_path))}
    b = {r["case"]: r for r in json.load(open(b_path))}
    assert set(a) == set(b), f"case sets differ: {set(a) ^ set(b)}"
    ok = True
    for case in sorted(a):
        r1, r2 = a[case]["rmse_total"], b[case]["rmse_total"]
        tol = max(0.15, 0.3 * max(r1, r2))
        line_ok = abs(r1 - r2) <= tol
        ok &= line_ok
        print(f"{case:<16} run1={r1:.3f} run2={r2:.3f} "
              f"tol={tol:.3f} {'OK' if line_ok else 'FAIL'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
