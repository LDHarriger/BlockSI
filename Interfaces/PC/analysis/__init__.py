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

from .cstr_k_d_model import (
    # Primary names (CSTR with decay)
    CSTRModel,
    fit_cstr_model,
    fit_fill_curve,
    fit_evac_curve,
    load_cstr_csv,
    load_cstr_model,
    load_cstr_model_from_dir,
    save_cstr_model,
    list_cstr_models,
    # Backward-compatible aliases
    FillModel,
    fit_fill_model,
    load_fill_csv,
    load_fill_model,
    load_fill_model_for_condition,
    save_fill_model,
    list_fill_models,
)

from .cstr_k_abs_model import (
    KAbsModel,
    fit_k_abs_model,
    save_k_abs_model,
    load_k_abs_model,
    load_k_abs_model_from_dir,
    load_k_abs_csv,
    list_k_abs_models,
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
    # CSTR model (with decay)
    "CSTRModel",
    "fit_cstr_model",
    "fit_fill_curve",
    "fit_evac_curve",
    "load_cstr_csv",
    "load_cstr_model",
    "load_cstr_model_from_dir",
    "save_cstr_model",
    "list_cstr_models",
    # Backward-compatible aliases
    "FillModel",
    "fit_fill_model",
    "load_fill_csv",
    "load_fill_model",
    "load_fill_model_for_condition",
    "save_fill_model",
    "list_fill_models",
    # k_abs model (loaded-vessel absorption)
    "KAbsModel",
    "fit_k_abs_model",
    "save_k_abs_model",
    "load_k_abs_model",
    "load_k_abs_model_from_dir",
    "load_k_abs_csv",
    "list_k_abs_models",
]
