import base64
import sys
from pathlib import Path


html_path = Path(sys.argv[1])
package_path = Path(sys.argv[2])
with html_path.open("r", encoding="utf-8", newline="") as source_file:
    html = source_file.read()

marker = "__MINIBCG_ASSET_PACKAGE__"
if html.count(marker) != 1 or not package_path.exists():
    raise SystemExit(f"Expected one asset package marker and package file for {html_path}")

package = base64.b64encode(package_path.read_bytes()).decode("ascii")
html_path.write_text(html.replace(marker, package), encoding="utf-8", newline="")
package_path.unlink()

for suffix in (".data", ".js", ".wasm"):
    html_path.with_suffix(suffix).unlink(missing_ok=True)
html_path.with_name(f"{html_path.stem}.worker.js").unlink(missing_ok=True)
