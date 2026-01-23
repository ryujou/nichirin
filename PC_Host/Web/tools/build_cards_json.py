import json
import random
import urllib.request
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

OUTPUT_FILE = Path(__file__).resolve().parent.parent / "card.json"
REGION = "jp"
START_ID = 1
END_ID = 99999
SUFFIXES = ["trim_normal.png", "trim_after_training.png"]
SAMPLE_SIZE = 100
RANDOM_SEED = 20260123
MAX_TRIES = 5000
WORKERS = 64
CHECK_TIMEOUT = 6


def is_valid_image(url: str) -> bool:
    try:
        req = urllib.request.Request(
            url,
            method="GET",
            headers={
                "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64)",
                "Accept": "image/*,*/*;q=0.8",
                "Range": "bytes=0-1023",
            },
        )
        with urllib.request.urlopen(req, timeout=CHECK_TIMEOUT) as resp:
            ctype = resp.headers.get("Content-Type", "")
            return ctype.startswith("image/")
    except Exception:
        return False


def build_urls() -> list[str]:
    rng = random.Random(RANDOM_SEED)
    urls: list[str] = []
    tries = 0
    while len(urls) < SAMPLE_SIZE and tries < MAX_TRIES:
        batch_size = min(800, MAX_TRIES - tries)
        candidates = []
        for _ in range(batch_size):
            tries += 1
            res_id = rng.randint(START_ID, END_ID)
            res = f"res{res_id:06d}_rip"
            suffix = rng.choice(SUFFIXES)
            url = f"https://bestdori.com/assets/{REGION}/characters/resourceset/{res}/{suffix}"
            if url in urls:
                continue
            candidates.append(url)

        if not candidates:
            continue

        with ThreadPoolExecutor(max_workers=WORKERS) as executor:
            future_map = {executor.submit(is_valid_image, url): url for url in candidates}
            for future in as_completed(future_map):
                url = future_map[future]
                try:
                    ok = future.result()
                except Exception:
                    ok = False
                if ok and url not in urls:
                    urls.append(url)
                    print(f"Valid {len(urls)}/{SAMPLE_SIZE}: {url}")
                if len(urls) >= SAMPLE_SIZE:
                    break

        if tries % 200 == 0:
            print(f"Tried {tries}/{MAX_TRIES}, valid={len(urls)}")
    print(f"Done: tried={tries}, valid={len(urls)}")
    return urls


def main() -> int:
    print(
        "Building urls: random valid images only, region=jp, "
        "suffixes=trim_normal.png + trim_after_training.png, "
        f"sample={SAMPLE_SIZE}, max_tries={MAX_TRIES}, workers={WORKERS}"
    )
    urls = build_urls()

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        json.dump({"cards": urls}, f, ensure_ascii=False, indent=2)

    print(f"Saved {len(urls)} urls to {OUTPUT_FILE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
