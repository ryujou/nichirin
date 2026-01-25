import json
import re
import urllib.parse
import urllib.request
from dataclasses import dataclass
from html.parser import HTMLParser
from pathlib import Path

USER_AGENT = "NichirinV3-AvatarFetcher/1.0"
API_URL = (
    "https://mzh.moegirl.org.cn/api.php?"
    "action=parse&prop=text&format=json&"
    "page=BanG_Dream!/%E7%99%BB%E5%9C%BA%E4%BA%BA%E7%89%A9%E5%8F%8A%E4%B9%90%E9%98%9F"
)

HEADER_COLOR = "\u4ee3\u8868\u8272"
HEADER_AVATAR = "\u5934\u50cf"
HEADER_NAME = "\u4e2d\u6587\u8bd1\u540d"
HEADER_BAND = "\u4e50\u961f"

ROOT_DIR = Path(__file__).resolve().parents[1]
OUTPUT_JSON = ROOT_DIR / "bangdream_avatars.json"
OUTPUT_DIR = ROOT_DIR / "assets" / "bangdream_avatars"


@dataclass
class TableCell:
    text: str
    img: str | None
    style: str
    is_header: bool


class WikiTableParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.tables: list[list[list[TableCell]]] = []
        self._in_table = False
        self._table_rows: list[list[TableCell]] = []
        self._in_tr = False
        self._current_row: list[TableCell] = []
        self._in_cell = False
        self._cell_text: list[str] = []
        self._cell_img: str | None = None
        self._cell_style = ""
        self._cell_is_header = False

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        attrs_map = dict(attrs)
        if tag == "table":
            if "wikitable" in (attrs_map.get("class") or ""):
                self._in_table = True
                self._table_rows = []
        elif self._in_table and tag == "tr":
            self._in_tr = True
            self._current_row = []
        elif self._in_table and self._in_tr and tag in ("th", "td"):
            self._in_cell = True
            self._cell_is_header = tag == "th"
            self._cell_text = []
            self._cell_img = None
            self._cell_style = attrs_map.get("style") or ""
        elif self._in_table and self._in_tr and self._in_cell and tag == "img":
            if not self._cell_img:
                src = attrs_map.get("src")
                if src:
                    self._cell_img = src
        elif self._in_table and self._in_tr and self._in_cell and tag == "br":
            self._cell_text.append("\n")

    def handle_endtag(self, tag: str) -> None:
        if tag == "table" and self._in_table:
            self.tables.append(self._table_rows)
            self._in_table = False
            self._table_rows = []
        elif self._in_table and tag == "tr":
            if self._in_tr and self._current_row:
                self._table_rows.append(self._current_row)
            self._in_tr = False
            self._current_row = []
        elif self._in_table and self._in_tr and tag in ("th", "td"):
            if self._in_cell:
                text = re.sub(r"\s+", " ", "".join(self._cell_text)).strip()
                self._current_row.append(
                    TableCell(
                        text=text,
                        img=self._cell_img,
                        style=self._cell_style,
                        is_header=self._cell_is_header,
                    )
                )
            self._in_cell = False
            self._cell_text = []
            self._cell_img = None
            self._cell_style = ""
            self._cell_is_header = False

    def handle_data(self, data: str) -> None:
        if self._in_table and self._in_tr and self._in_cell:
            self._cell_text.append(data)


def fetch_html() -> str:
    req = urllib.request.Request(API_URL, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=30) as resp:
        payload = json.loads(resp.read().decode("utf-8", errors="replace"))
    return payload["parse"]["text"]["*"]


def normalize_header(text: str) -> str:
    return re.sub(r"\s+", "", text)


def find_hex_color(text: str, style: str) -> str | None:
    match = re.search(r"#(?:[0-9a-fA-F]{6})", text)
    if match:
        return match.group(0).upper()
    match = re.search(r"#(?:[0-9a-fA-F]{6})", style)
    if match:
        return match.group(0).upper()
    return None


def clean_name(text: str) -> str:
    cleaned = re.sub(r"\[\d+\]", "", text)
    cleaned = re.sub(r"\s+", "", cleaned)
    return cleaned.strip()


def clean_label(text: str) -> str:
    cleaned = re.sub(r"\[\d+\]", "", text)
    cleaned = re.sub(r"\s+", " ", cleaned)
    return cleaned.strip()


def slugify(text: str) -> str:
    slug = re.sub(r"[^0-9A-Za-z]+", "_", text).strip("_")
    return slug


def image_extension(url: str) -> str:
    path = urllib.parse.urlparse(url).path
    base = path.split("!")[0]
    ext = Path(base).suffix.lower()
    if ext in {".png", ".jpg", ".jpeg", ".webp", ".gif"}:
        return ext
    return ".png"


def extract_entries(html: str) -> list[dict]:
    parser = WikiTableParser()
    parser.feed(html)
    entries: list[dict] = []
    for table in parser.tables:
        header_row_index = None
        header_map: dict[str, int] = {}
        for idx, row in enumerate(table):
            if not row:
                continue
            if any(cell.is_header for cell in row):
                normalized = [normalize_header(cell.text) for cell in row]
                if HEADER_COLOR in normalized and HEADER_AVATAR in normalized and HEADER_NAME in normalized:
                    header_row_index = idx
                    header_map = {normalized[i]: i for i in range(len(normalized))}
                    break
        if header_row_index is None:
            continue
        color_idx = header_map[HEADER_COLOR]
        avatar_idx = header_map[HEADER_AVATAR]
        name_idx = header_map[HEADER_NAME]
        band_idx = header_map.get(HEADER_BAND)
        for row in table[header_row_index + 1 :]:
            if not row or len(row) <= max(color_idx, avatar_idx, name_idx):
                continue
            color = find_hex_color(row[color_idx].text, row[color_idx].style)
            avatar_url = row[avatar_idx].img
            name = clean_name(row[name_idx].text)
            band = clean_label(row[band_idx].text) if band_idx is not None and len(row) > band_idx else ""
            if not name or not avatar_url:
                continue
            entries.append(
                {
                    "name": name,
                    "color": color,
                    "band": band,
                    "avatar_url": avatar_url,
                }
            )
    return entries


def download_image(url: str, path: Path) -> None:
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=30) as resp:
        data = resp.read()
    path.write_bytes(data)


def main() -> int:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    html = fetch_html()
    entries = extract_entries(html)
    if not entries:
        raise SystemExit("No entries parsed. Check page structure.")

    results: list[dict] = []
    failures = 0
    for idx, entry in enumerate(entries, start=1):
        ext = image_extension(entry["avatar_url"])
        slug = slugify(entry["name"]) or slugify(entry["color"] or "") or f"item{idx:03d}"
        filename = f"{idx:03d}_{slug}{ext}"
        local_path = OUTPUT_DIR / filename
        if not local_path.exists():
            try:
                download_image(entry["avatar_url"], local_path)
            except Exception as exc:
                failures += 1
                print(f"Download failed: {entry['name']} -> {exc}")
                continue
        results.append(
            {
                "name": entry["name"],
                "color": entry["color"],
                "band": entry["band"],
                "image": str(Path("assets") / "bangdream_avatars" / filename).replace("\\", "/"),
                "source_url": entry["avatar_url"],
            }
        )

    with open(OUTPUT_JSON, "w", encoding="utf-8") as f:
        json.dump({"items": results}, f, ensure_ascii=False, indent=2)

    print(f"Saved {len(results)} items to {OUTPUT_JSON}")
    if failures:
        raise SystemExit(f"{failures} downloads failed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
