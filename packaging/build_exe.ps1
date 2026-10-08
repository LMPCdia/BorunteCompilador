# Construye dist/BorunteDSL.exe y NO lo da por bueno hasta que pase el
# self-test sobre el binario ya construido.
#
# El self-test es el criterio de aceptacion, no un extra: compiler/grammar.lark
# se lee como archivo en tiempo de ejecucion, asi que un empaquetado incompleto
# produce un .exe que abre bien y falla al compilar el primer programa. Con
# console=False ese error no se ve en ninguna parte.
#
# Uso:
#   .\packaging\build_exe.ps1
#   .\packaging\build_exe.ps1 -SkipTests
#
# EL .EXE HAY QUE CONSTRUIRLO EN WINDOWS: PyInstaller no compila cruzado.

param(
    [switch]$SkipTests
)

$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

# Usar el venv del proyecto si existe.
$python = Join-Path $root "venv\Scripts\python.exe"
if (-not (Test-Path $python)) {
    $python = "python"
}
Write-Host "Python: $python"

if (-not $SkipTests) {
    Write-Host "`n=== Bateria de tests ===" -ForegroundColor Cyan
    $env:QT_QPA_PLATFORM = "offscreen"
    & $python -m pytest tests/ -q
    if ($LASTEXITCODE -ne 0) {
        throw "Los tests fallaron: no se construye el ejecutable."
    }
}

Write-Host "`n=== Dependencias de build ===" -ForegroundColor Cyan
& $python -m pip install -q -r (Join-Path $PSScriptRoot "requirements-build.txt")
if ($LASTEXITCODE -ne 0) { throw "No se pudieron instalar las dependencias de build." }

# Limpieza del workpath ANTES de invocar, y no con --clean.
#
# Con --clean, PyInstaller hace el rmtree el mismo, y si Windows le niega el
# acceso a un solo directorio (un antivirus escaneando el arbol mientras lo
# recorre alcanza) aborta el build entero con un PermissionError. Borrarlo desde
# acá y tolerar el fallo deja que el build siga: un workpath sucio no rompe nada,
# PyInstaller lo sobreescribe.
$buildDir = Join-Path $root "build"
if (Test-Path $buildDir) {
    Write-Host "`nLimpiando $buildDir"
    for ($intento = 1; $intento -le 3; $intento++) {
        try {
            Remove-Item -Recurse -Force $buildDir -ErrorAction Stop
            break
        } catch {
            if ($intento -eq 3) {
                Write-Host "  no se pudo borrar del todo; se construye igual: $($_.Exception.Message)" -ForegroundColor Yellow
            } else {
                Start-Sleep -Milliseconds 400
            }
        }
    }
}

Write-Host "`n=== PyInstaller ===" -ForegroundColor Cyan
& $python -m PyInstaller (Join-Path $PSScriptRoot "BorunteDSL.spec") --noconfirm
if ($LASTEXITCODE -ne 0) { throw "PyInstaller fallo." }

$exe = Join-Path $root "dist\BorunteDSL.exe"
if (-not (Test-Path $exe)) { throw "No se genero $exe" }

$sizeMb = [math]::Round((Get-Item $exe).Length / 1MB, 1)
Write-Host "`nGenerado: $exe ($sizeMb MB)"

Write-Host "`n=== Self-test sobre el ejecutable ===" -ForegroundColor Cyan

# Start-Process -Wait y no "& $exe": el ejecutable se construye con
# console=False (subsistema Windows), asi que PowerShell NO lo espera y
# $LASTEXITCODE queda en 0 pase lo que pase. Sin esto, el criterio de
# aceptacion daria verde siempre, que es peor que no tenerlo.
#
# La redireccion tambien cumple una funcion: le da a la app un stdout valido.
# Sin un handle, PyInstaller en modo windowed deja sys.stdout en None.
$outFile = Join-Path $env:TEMP "borunte_selftest_out.txt"
$errFile = Join-Path $env:TEMP "borunte_selftest_err.txt"
$proc = Start-Process -FilePath $exe -ArgumentList "--self-test" -Wait -PassThru `
    -RedirectStandardOutput $outFile -RedirectStandardError $errFile

if (Test-Path $outFile) { Get-Content $outFile | Write-Host }
if ((Test-Path $errFile) -and (Get-Item $errFile).Length -gt 0) {
    Write-Host "--- stderr ---" -ForegroundColor Yellow
    Get-Content $errFile | Write-Host
}

if ($proc.ExitCode -ne 0) {
    throw "El self-test del ejecutable fallo (exit $($proc.ExitCode)). El .exe NO sirve: lo mas probable es que falte un archivo de datos (ver packaging/datafiles.py)."
}

Write-Host "`nListo. $exe paso el self-test." -ForegroundColor Green
