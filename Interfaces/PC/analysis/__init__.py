"""BlockSI analysis package — calibration data processing and model fitting."""

from .power_o3_model import (
    PowerO3Model,
    fit_sigmoid_model,
    load_model,
    load_model_for_condition,
    save_model,
    list_models,
    load_calibration_csv,
    aggregate_calibration_data,
    predict_o3,
    predict_power,
    generate_curve,
)

from .fill_model import (
    FillModel,
    fit_fill_model,
    fit_fill_curve,
    fit_evac_curve,
    load_fill_csv,
    load_fill_model,
    load_fill_model_for_condition,
    save_fill_model,
    list_fill_models,
)

# Re-export list_calibration_files as utility
from .power_o3_model import list_models as list_models

__all__ = [
    # Power-O3 sigmoid model
    "PowerO3Model",
    "fit_sigmoid_model",
    "load_model",
    "load_model_for_condition",
    "save_model",
    "list_models",
    "load_calibration_csv",
    "aggregate_calibration_data",
    "predict_o3",
    "predict_power",
    "generate_curve",
    # Fill / Evacuation CSTR model
    "FillModel",
    "fit_fill_model",
    "fit_fill_curve",
    "fit_evac_curve",
    "load_fill_csv",
    "load_fill_model",
    "load_fill_model_for_condition",
    "save_fill_model",
    "list_fill_models",
]
