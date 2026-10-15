import json, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.special import erfc
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": 0.3})
res = json.load(open("build/ber.json"))
fig, ax = plt.subplots(figsize=(6.4, 4.4))
e = np.linspace(4, 19, 200); es = 10 ** (e / 10)
ax.semilogy(e, 0.5 * erfc(np.sqrt(es / 2)), color="#1f5fa6", lw=1, label="QPSK theory")
ax.semilogy(e, 3 / 8 * erfc(np.sqrt(es / 10)), color="#b5432a", lw=1, label="16-QAM theory")
for R, mk in ((8, "o"), (40, "s")):
    for mod, col in ((0, "#1f5fa6"), (1, "#b5432a")):
        pts = sorted([(r["ESN0"], r["ber"]) for r in res if r["R"] == R and r["MOD"] == mod and r["ber"]])
        ax.semilogy(*zip(*pts), mk, color=col, mfc="none", ms=6,
                    label=f"RTL {'16-QAM' if mod else 'QPSK'}, {40/R:.0f} MHz ch.")
ax.set_ylim(5e-5, 0.1); ax.set_xlim(4, 19)
ax.set_xlabel("Es/N0 (dB)"); ax.set_ylabel("payload BER")
ax.set_title("nb_radio RTL loopback vs theory (CFO 5 kHz, SRO 10 ppm, uncoded)")
ax.legend(fontsize=8, loc="lower left")
fig.tight_layout(); fig.savefig("docs/ber.png", dpi=140)

d = np.loadtxt("build/syms_16qam.txt"); y = d[:, 0] + 1j * d[:, 1]; st = d[:, 4]; m16 = d[:, 3]
fig, axs = plt.subplots(1, 2, figsize=(8, 4))
for a, sel, t in ((axs[0], (st == 0)[-4000:], "idle (QPSK)"), (axs[1], ((st == 3) & (m16 == 1))[-4000:], "payload (16-QAM)")):
    yy = y[-4000:][sel]
    a.plot(yy.real, yy.imag, ".", ms=1.5, color="#1f5fa6", alpha=0.5)
    a.set_aspect("equal"); a.set_xlim(-16000, 16000); a.set_ylim(-16000, 16000); a.set_title(t)
fig.suptitle("Equalizer output, 5 MHz channel, Es/N0 25 dB, CFO 20 kHz, SRO 20 ppm, 2-path echo")
fig.tight_layout(); fig.savefig("docs/constellation.png", dpi=140)
print("ok")
