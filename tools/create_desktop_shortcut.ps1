param(
    [Parameter(Mandatory = $true)][string]$PythonPath,
    [string]$RosSetup = '',
    [string]$ShortcutPath = (Join-Path ([Environment]::GetFolderPath('Desktop')) 'MMHand.lnk')
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$python = (Resolve-Path -LiteralPath $PythonPath).Path
$setup = if ($RosSetup) { (Resolve-Path -LiteralPath $RosSetup).Path } else { '' }
# Paths become cmd arguments. Reject expansion/control characters before writing.
foreach ($path in @($root, $python, $setup)) {
    if ($path -match '[%"\r\n]') { throw 'Launcher paths cannot contain %, quotes or newlines.' }
}
if (-not (Test-Path -LiteralPath $python -PathType Leaf) -or
    ($setup -and -not (Test-Path -LiteralPath $setup -PathType Leaf))) {
    throw 'PythonPath and RosSetup must point to files.'
}
$shell = New-Object -ComObject WScript.Shell
$shortcut = $shell.CreateShortcut([IO.Path]::GetFullPath($ShortcutPath))
$shortcut.TargetPath = $env:ComSpec
$shortcut.Arguments = '/d /v:off /s /c "set "MMHAND_PYTHON={0}" & set "MMHAND_ROS_SETUP={1}" & call "{2}\start_mmhand.cmd""' -f $python, $setup, $root
$shortcut.WorkingDirectory = $root
$shortcut.IconLocation = "$python,0"
$shortcut.WindowStyle = 7
$shortcut.Description = 'MMHand Desktop (ROS2 enabled; motion requires authorization)'
$shortcut.Save()
Write-Output "Created $ShortcutPath"
