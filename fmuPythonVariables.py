from fmpy import read_model_description

def list_variables(fmu_path):
    desc = read_model_description(fmu_path)
    print(f"\n--- {fmu_path} ---")
    for var in desc.modelVariables:
        print(f"{var.name:15} causality={var.causality:10} type={var.type}")

#list_variables("InvertedPendulum.fmu")      # altes Modell
list_variables("InvertedPendulumMB.fmu")    # neues MultiBody-Modell