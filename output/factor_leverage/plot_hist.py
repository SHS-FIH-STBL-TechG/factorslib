import csv, sys, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def read_values(csv_path, value_col, status_col):
    active = []
    suppressed = []
    with open(csv_path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        headers = reader.fieldnames or []
        has_status = status_col and status_col in headers
        if value_col not in headers:
            return active, suppressed
        for row in reader:
            try:
                v = float(row[value_col])
                if not math.isfinite(v):
                    continue
                if has_status:
                    status = row.get(status_col, "").strip().lower()
                    if status == "suppressed":
                        suppressed.append(v)
                    else:
                        active.append(v)
                else:
                    active.append(v)
            except Exception:
                pass
    return active, suppressed

def build_bins(values, bin_count):
    vmin = min(values)
    vmax = max(values)
    if math.isclose(vmin, vmax, rel_tol=1e-9, abs_tol=1e-12):
        span = abs(vmin) if abs(vmin) > 1e-6 else 1.0
        vmin -= span * 0.5
        vmax += span * 0.5
    return np.linspace(vmin, vmax, bin_count + 1)

def main():
    if len(sys.argv) < 5:
        print("Usage: python plot_hist.py data.csv output.png title column_name [subtitle] [status_column]")
        return 1
    csv_path, png_path, title, column_name = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
    subtitle = sys.argv[5] if len(sys.argv) >= 6 else ""
    status_col = sys.argv[6] if len(sys.argv) >= 7 else ""
    active, suppressed = read_values(csv_path, column_name, status_col)
    combined = active + suppressed
    if not combined:
        print(f"[WARN] no values found in {csv_path}")
        return 0
    plt.figure(figsize=(8, 4.5))
    bins = 50
    bin_edges = build_bins(combined, bins)
    has_any = False
    if suppressed:
        plt.hist(suppressed, bins=bin_edges, color="#b0b0b0", alpha=0.95, edgecolor="white", label="|z| ≤ θ")
        has_any = True
    if active:
        plt.hist(active, bins=bin_edges, color="#1f77b4", alpha=0.85, edgecolor="white", label="|z| > θ")
        has_any = True
    if suppressed and active:
        plt.legend()
    if not has_any:
        print(f"[WARN] no values found in {csv_path}")
        return 0
    plt.title(title)
    plt.xlabel("value")
    plt.ylabel("frequency")
    plt.grid(alpha=0.3)
    if subtitle:
        plt.figtext(0.5, 0.02, subtitle, ha="center", fontsize=9)
        plt.tight_layout(rect=[0, 0.05, 1, 1])
    else:
        plt.tight_layout()
    plt.savefig(png_path)
    return 0

if __name__ == "__main__":
    sys.exit(main())
