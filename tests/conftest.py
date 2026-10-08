"""Configuración común de los tests."""

import pytest
from PySide6.QtCore import QSettings


@pytest.fixture(autouse=True, scope="session")
def _isolated_settings(tmp_path_factory):
    # La app recuerda cosas (último modelo de robot) con QSettings: que los
    # tests no lean ni escriban la configuración real del usuario.
    QSettings.setPath(QSettings.Format.IniFormat, QSettings.Scope.UserScope,
                      str(tmp_path_factory.mktemp("settings")))


@pytest.fixture(autouse=True)
def _isolated_user_models(tmp_path_factory, monkeypatch):
    # Ni los modelos de robot que el usuario tenga en ~/BorunteDSL/modelos
    # (uno roto haría fallar tests que no tienen nada que ver).
    import sim.kinematics as kinematics

    monkeypatch.setattr(kinematics, "USER_MODELS_DIR", tmp_path_factory.mktemp("modelos"))
