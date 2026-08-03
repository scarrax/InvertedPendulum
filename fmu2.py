from vergleichMBvsNormal import simulate
print("Teste InvertedPendulumMB.fmu ...")
df = simulate("InvertedPendulumMB.fmu", duration=2.0)
print("OK, Zeilen:", len(df))