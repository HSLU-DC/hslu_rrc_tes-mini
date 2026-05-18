# Grasshopper Python Component: Holzbedarf aus fab_data
# ghenv.Component.Message = 'Holzbedarf v1'
#
# Zaehlt Elemente pro stock_category und summiert die benoetigten
# Stock-Laufmeter (= was bestellt werden muss).
#
# ==============================================================================
# INPUTS
# ==============================================================================
#   fab_data (DataTree):  Tree mit {layer;element} Struktur,
#                         stock_category an Index 11
#                         Type Hint: GenericObject, Access: tree
#
# ==============================================================================
# OUTPUTS
# ==============================================================================
#   summary (str):  Mehrzeiliger Bedarfs-Report -- auf ein Panel verbinden

VALID_STOCK_CATEGORIES = ("400", "550", "750", "1000")
STOCK_CATEGORY_INDEX = 11

# Stock-Laenge pro Kategorie in Meter
STOCK_LENGTHS_M = {"400": 0.40, "550": 0.55, "750": 0.75, "1000": 1.00}

counts = {s: 0 for s in VALID_STOCK_CATEGORIES}
unknown = 0
total = 0

if fab_data is not None:
    for path in fab_data.Paths:
        branch = list(fab_data.Branch(path))
        if STOCK_CATEGORY_INDEX < len(branch) and branch[STOCK_CATEGORY_INDEX] is not None:
            size = str(branch[STOCK_CATEGORY_INDEX]).strip().strip('"').strip("'")
            if size in counts:
                counts[size] += 1
            else:
                unknown += 1
            total += 1

lm_total = sum(counts[s] * STOCK_LENGTHS_M[s] for s in VALID_STOCK_CATEGORIES)

lines = ["Holzbedarf:"]
for s in VALID_STOCK_CATEGORIES:
    lm = counts[s] * STOCK_LENGTHS_M[s]
    lines.append("  {} mm: {:>3} Stueck = {:>5.2f} lm".format(s, counts[s], lm))
if unknown:
    lines.append("  ?:       {:>3} Stueck (unbekannte Groesse!)".format(unknown))
lines.append("  " + "-" * 28)
lines.append("  Total:   {:>3} Stueck = {:>5.2f} lm".format(total, lm_total))

summary = "\n".join(lines)
