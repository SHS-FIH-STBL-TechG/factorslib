import csv, sys, math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

def read_values(csv_path):
    values = []
    with open(csv_path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                v = float(row["value"])
                if math.isfinite(v):
                    values.append(v)
            except Exception:
                pass
    return values

def main():
    if len(sys.argv) < 4:
        print("Usage: python plot_hist.py data.csv output.png title")
        return 1
    csv_path, png_path, title = sys.argv[1], sys.argv[2], sys.argv[3]
    values = read_values(csv_path)
    if not values:
        print(f"[WARN] no values found in {csv_path}")
        return 0
    plt.figure(figsize=(8, 4.5))
    plt.hist(values, bins=50, color="#1f77b4", alpha=0.85, edgecolor="white")
    plt.title(title)
    plt.xlabel("value")
    plt.ylabel("frequency")
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(png_path)
    return 0

if __name__ == "__main__":
    sys.exit(main())
