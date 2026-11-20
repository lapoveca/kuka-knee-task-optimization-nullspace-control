from pathlib import Path
import ast

path = Path("review_vv_campaign.py")
source = path.read_text()
changes = [
    ("actual=FK@A;target=", "actual_BT=FK@A;target="),
    ("actual[:,:3,3]-BT[:,:3,3]", "actual_BT[:,:3,3]-BT[:,:3,3]"),
    ("@actual[:,:3,:3]", "@actual_BT[:,:3,:3]"),
    ("desired@np.linalg.inv(actual),'E required'", "desired@np.linalg.inv(A),'E required'"),
    ("desired@np.linalg.inv(actual)@np.linalg.inv(task.pose_at(0)),'F'", "BF,'F'"),
    ("for ax,(close,camera,label) in zip(axs,views):scene(ax,qi[k],desired,actual,ee,label,close,camera)",
     "for ax,(close,camera,view_label) in zip(axs,views):scene(ax,qi[k],desired,actual,ee,view_label,close,camera)"),
]
for old, new in changes:
    count = source.count(old)
    if count != 1:
        raise SystemExit(f"Expected exactly one occurrence, found {count}: {old!r}. No file changed.")
    source = source.replace(old, new, 1)
ast.parse(source)
compile(source, str(path), "exec")
path.write_text(source)
print("Patched reviewer only: array naming, required EE/F frames, and GIF label scope.")
