"""Bundle a results directory into a single self-contained HTML dashboard.

Embeds every PNG as base64 (so the file is portable) and renders report.md +
the EKF-replay metrics. Usage: python3 make_report_html.py <results_dir> [bag_name]
"""
import base64, glob, html, os, re, sys

OUT = sys.argv[1]
BAG = sys.argv[2] if len(sys.argv) > 2 else os.path.basename(OUT.rstrip("/"))


def b64(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode()


def img(path, w="100%"):
    if not os.path.exists(path):
        return ""
    cap = html.escape(os.path.basename(path))
    return (f'<figure><img style="width:{w};max-width:1100px" '
            f'src="data:image/png;base64,{b64(path)}"/>'
            f'<figcaption>{cap}</figcaption></figure>')


# ---- minimal markdown -> html (headings, fenced code, tables, bold, lists) ----
def md2html(md):
    out, i, lines = [], 0, md.split("\n")
    while i < len(lines):
        ln = lines[i]
        if ln.startswith("```"):
            buf = []
            i += 1
            while i < len(lines) and not lines[i].startswith("```"):
                buf.append(html.escape(lines[i])); i += 1
            out.append("<pre class='code'>" + "\n".join(buf) + "</pre>"); i += 1; continue
        if re.match(r"^\|.*\|\s*$", ln):                       # table block
            tbl = []
            while i < len(lines) and re.match(r"^\|.*\|\s*$", lines[i]):
                tbl.append(lines[i]); i += 1
            out.append(render_table(tbl)); continue
        if ln.startswith("#"):
            n = len(ln) - len(ln.lstrip("#"))
            out.append(f"<h{n}>{inline(ln.lstrip('#').strip())}</h{n}>"); i += 1; continue
        if re.match(r"^\s*[-*]\s+", ln):
            items = []
            while i < len(lines) and re.match(r"^\s*[-*]\s+", lines[i]):
                items.append("<li>" + inline(re.sub(r"^\s*[-*]\s+", "", lines[i])) + "</li>"); i += 1
            out.append("<ul>" + "".join(items) + "</ul>"); continue
        if ln.strip() == "":
            i += 1; continue
        out.append("<p>" + inline(ln) + "</p>"); i += 1
    return "\n".join(out)


def inline(s):
    s = html.escape(s)
    s = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", s)
    s = re.sub(r"`(.+?)`", r"<code>\1</code>", s)
    return s


def render_table(rows):
    cells = [[c.strip() for c in r.strip().strip("|").split("|")] for r in rows]
    cells = [r for r in cells if not all(set(c) <= set("-: ") for c in r)]
    if not cells:
        return ""
    head = "".join(f"<th>{inline(c)}</th>" for c in cells[0])
    body = "".join("<tr>" + "".join(f"<td>{inline(c)}</td>" for c in r) + "</tr>" for r in cells[1:])
    return f"<table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>"


def read(path):
    return open(path).read() if os.path.exists(path) else ""


# ---- assemble ----
parts = [f"""<!doctype html><meta charset=utf-8><title>odom_calib — {html.escape(BAG)}</title>
<style>
 body{{font-family:-apple-system,Segoe UI,Roboto,sans-serif;max-width:1180px;margin:24px auto;padding:0 18px;color:#1b1b1b;line-height:1.5}}
 h1{{border-bottom:3px solid #2b6cb0;padding-bottom:6px}} h2{{margin-top:34px;color:#2b6cb0;border-bottom:1px solid #ddd}}
 table{{border-collapse:collapse;margin:10px 0;font-size:14px}} th,td{{border:1px solid #ccc;padding:5px 10px;text-align:right}}
 th:first-child,td:first-child{{text-align:left}} thead{{background:#eef4fb}}
 code{{background:#f0f0f0;padding:1px 4px;border-radius:3px;font-size:90%}} pre.code{{background:#1e1e2e;color:#e0e0e0;padding:12px;border-radius:6px;overflow:auto;font-size:13px}}
 figure{{margin:14px 0;text-align:center}} figcaption{{color:#666;font-size:12px}} img{{border:1px solid #ddd;border-radius:6px}}
 .nav a{{margin-right:14px}} .note{{background:#fff8e1;border-left:4px solid #f0b400;padding:8px 12px}}
</style>
<h1>odom_calib report — {html.escape(BAG)}</h1>
<div class=nav><b>Jump:</b>
 <a href='#cal'>Calibration &amp; report</a>
 <a href='#fig'>Offline figures (1–7)</a>
 <a href='#ekf'>robot_localization replay</a></div>
"""]

# 1. report.md
rep = read(os.path.join(OUT, "report.md"))
parts.append("<h2 id='cal'>1. Calibration &amp; dead-reckoning report</h2>")
parts.append(md2html(rep) if rep else "<p class=note>report.md not found.</p>")

# 2. offline figures
parts.append("<h2 id='fig'>2. Offline analysis figures</h2>")
figs = sorted(glob.glob(os.path.join(OUT, "0*.png")) + glob.glob(os.path.join(OUT, "07_*.png")))
figs = sorted(set(figs))
if figs:
    for f in figs:
        parts.append(img(f))
else:
    parts.append("<p class=note>no offline figures found.</p>")

# 3. EKF replay
parts.append("<h2 id='ekf'>3. robot_localization replay (vesc/odom + IMU AHRS)</h2>")
ekf_dir = os.path.join(OUT, "ekf_replay")
metrics = read(os.path.join(ekf_dir, "metrics.txt"))
if metrics:
    parts.append("<pre class='code'>" + html.escape(metrics) + "</pre>")
ekf_figs = sorted(glob.glob(os.path.join(ekf_dir, "*.png")))
if ekf_figs:
    for f in ekf_figs:
        parts.append(img(f))
else:
    parts.append("<p class=note>EKF replay was skipped (run with ROS sourced to enable). "
                 "Offline figures already emulate this — see <code>ahrs_gyro</code>.</p>")

with open(os.path.join(OUT, "index.html"), "w") as f:
    f.write("\n".join(parts))
print("wrote", os.path.join(OUT, "index.html"))
