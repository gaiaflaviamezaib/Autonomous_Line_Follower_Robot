from pathlib import Path


def main():
    root = Path(__file__).resolve().parent
    failures = []

    for path in sorted(root.glob("*.py")):
        if path.name == Path(__file__).name:
            continue
        try:
            compile(path.read_text(encoding="utf-8"), str(path), "exec")
            print(f"OK   {path.name}")
        except Exception as exc:
            failures.append((path.name, exc))
            print(f"FAIL {path.name}")
            print(exc)

    if failures:
        raise SystemExit(1)

    print("All Python files compiled successfully.")


if __name__ == "__main__":
    main()
