from __future__ import annotations

import os
from pathlib import Path

from pwr_pt_rl_control import build_default_case, run_simulation
from pwr_pt_rl_control.plotting import create_plots
from pwr_pt_rl_control.simulation import ensure_output_dir


def main() -> None:
    project_root = Path(__file__).resolve().parent
    case = build_default_case()
    result = run_simulation(case)
    output_dir = ensure_output_dir(project_root)
    show_plots = os.environ.get("PWR_SHOW_PLOTS", "").lower() in {"1", "true", "yes"}
    create_plots(result, case, output_dir=output_dir, show=show_plots)

    final_power = result.plant_state[-1, 0]
    final_error = result.reference[-1] - final_power
    print("Simulation completed.")
    print(f"Final reference power: {result.reference[-1]:.4f} FP")
    print(f"Final actual power:    {final_power:.4f} FP")
    print(f"Final tracking error:  {final_error:.4e} FP")
    print(f"Plots saved in:        {output_dir}")


if __name__ == "__main__":
    main()
