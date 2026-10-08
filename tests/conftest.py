"""Configuración común de los tests."""

import pytest
from PySide6.QtCore import QSettings


@pytest.fixture(autouse=True, scope="session")
def _isolated_settings(tmp_path_factory):
    # La app recuerda cosas (último modelo de robot) con QSettings: que los
    # tests no lean ni escriban la configuración real del usuario.
    QSettings.setPath(QSettings.Format.IniFormat, QSettings.Scope.UserScope,
                      str(tmp_path_factory.mktemp("settings")))
