param(
    [string]$P4Port = "",
    [string]$S3Port = "",
    [string]$Python = "",
    [switch]$DryRun,
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ExtraArgs
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$MainPy = Join-Path $ScriptDir "main.py"

if (-not (Test-Path -LiteralPath $MainPy)) {
    throw "Cannot find desktop main.py: $MainPy"
}

if (-not $P4Port -and $env:P4_PORT) {
    $P4Port = $env:P4_PORT
}
if (-not $S3Port -and $env:S3_PORT) {
    $S3Port = $env:S3_PORT
}
if (-not $S3Port -and $env:TACTILE_PORT) {
    $S3Port = $env:TACTILE_PORT
}

function Resolve-PythonCommand {
    param([string]$Requested)

    if ($Requested) {
        return $Requested
    }
    if ($env:PYTHON) {
        return $env:PYTHON
    }
    foreach ($candidate in @("python", "py")) {
        $cmd = Get-Command $candidate -ErrorAction SilentlyContinue
        if ($cmd) {
            return $candidate
        }
    }
    throw "Cannot find python. Install Python or pass -Python C:\path\to\python.exe"
}

$PythonCommand = Resolve-PythonCommand $Python

$DetectorCode = @'
import json
import re
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except Exception as exc:
    print(f"pyserial is required for auto-detection: {exc}", file=sys.stderr)
    raise SystemExit(2)

BAUDRATE = 921600
PROTOCOL_HEADER = 0xFE
PROTOCOL_TAIL = 0xFF
CONTROL_PACKET_TYPES = {0x01, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0B, 0x0C, 0x0D, 0x0E}
TACTILE_SUMMARY_PACKET_TYPES = {0x0A}
TACTILE_FULL_PACKET_TYPES = {0x0E, 0x0F}
TACTILE_FULL_PACKET_MAGIC = b"T2"

p4_override = (sys.argv[1] if len(sys.argv) > 1 else "").strip()
s3_override = (sys.argv[2] if len(sys.argv) > 2 else "").strip()


def com_sort_key(port):
    match = re.search(r"(\d+)$", port or "")
    return (int(match.group(1)) if match else 9999, port or "")


def vidpid(port_info):
    return getattr(port_info, "vid", None), getattr(port_info, "pid", None)


def port_text(port_info):
    parts = (
        getattr(port_info, "device", "") or "",
        getattr(port_info, "description", "") or "",
        getattr(port_info, "hwid", "") or "",
        getattr(port_info, "manufacturer", "") or "",
        getattr(port_info, "product", "") or "",
    )
    return " ".join(parts).lower()


def is_s3_candidate(port_info):
    vid, _pid = vidpid(port_info)
    text = port_text(port_info)
    return (
        vid == 0x303A
        or "vid:pid=303a" in text
        or "esp32" in text
        or "espressif" in text
    )


def is_ch34x_candidate(port_info):
    vid, _pid = vidpid(port_info)
    text = port_text(port_info)
    return (
        vid == 0x1A86
        or "vid:pid=1a86" in text
        or "ch343" in text
        or "ch340" in text
        or "usb-enhanced-serial" in text
    )


def protocol_score(port_name):
    try:
        ser = serial.Serial()
        ser.port = port_name
        ser.baudrate = BAUDRATE
        ser.timeout = 0.05
        ser.write_timeout = 0.05
        ser.dtr = False
        ser.rts = False
        ser.open()
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        return {"control": -1, "tactile": -1, "total": -1}

    try:
        deadline = time.time() + 0.45
        buffer = bytearray()
        while time.time() < deadline:
            waiting = ser.in_waiting
            if waiting:
                buffer.extend(ser.read(waiting))
            else:
                time.sleep(0.02)

        control = 0
        tactile = 0
        total = 0
        index = 0
        while index + 3 < len(buffer):
            if buffer[index] != PROTOCOL_HEADER:
                index += 1
                continue
            length = buffer[index + 1]
            end = index + 2 + length
            if end >= len(buffer):
                break
            if buffer[end] != PROTOCOL_TAIL:
                index += 1
                continue

            packet_type = buffer[index + 2]
            payload = bytes(buffer[index + 3:end])
            is_tactile_full = (
                packet_type in TACTILE_FULL_PACKET_TYPES
                and payload.startswith(TACTILE_FULL_PACKET_MAGIC)
            )
            total += 1
            if packet_type in CONTROL_PACKET_TYPES and not is_tactile_full:
                control += 1
            if packet_type in TACTILE_SUMMARY_PACKET_TYPES or is_tactile_full:
                tactile += 1
            index = end + 1
        return {"control": control, "tactile": tactile, "total": total}
    finally:
        try:
            ser.close()
        except Exception:
            pass


ports = sorted(list(list_ports.comports()), key=lambda item: com_sort_key(item.device))
if not ports:
    print("no serial ports found", file=sys.stderr)
    raise SystemExit(3)

scores = {port.device: protocol_score(port.device) for port in ports}
s3_candidates = [port for port in ports if is_s3_candidate(port)]
ch34x_candidates = [port for port in ports if is_ch34x_candidate(port)]

s3_port = s3_override
if not s3_port:
    scored_tactile = sorted(
        ((scores[port.device]["tactile"], port.device) for port in ports),
        key=lambda item: (-item[0], com_sort_key(item[1])),
    )
    if scored_tactile and scored_tactile[0][0] > 0:
        s3_port = scored_tactile[0][1]
    elif s3_candidates:
        s3_port = s3_candidates[0].device

p4_port = p4_override
if not p4_port:
    candidates = [port for port in ports if port.device != s3_port]
    control_candidates = [port for port in candidates if is_ch34x_candidate(port)] or candidates
    scored_control = sorted(
        ((scores[port.device]["control"], port.device) for port in control_candidates),
        key=lambda item: (-item[0], com_sort_key(item[1])),
    )
    if scored_control:
        p4_port = scored_control[0][1]

if not p4_port:
    print("could not detect P4/control port. Use -P4Port COMx.", file=sys.stderr)
    raise SystemExit(4)
if not s3_port:
    print("could not detect S3/tactile port. Use -S3Port COMx.", file=sys.stderr)
    raise SystemExit(5)
if p4_port == s3_port and len(ports) > 1:
    print(
        f"auto-detection selected the same port for P4 and S3 ({p4_port}). "
        "Use -P4Port COMx -S3Port COMy.",
        file=sys.stderr,
    )
    raise SystemExit(6)

out_ports = []
for port in ports:
    vid, pid = vidpid(port)
    vidpid_text = f"{vid:04X}:{pid:04X}" if vid is not None and pid is not None else "----:----"
    out_ports.append(
        {
            "device": port.device,
            "vidpid": vidpid_text,
            "description": port.description or "",
            "hwid": port.hwid or "",
            "control_score": scores[port.device]["control"],
            "tactile_score": scores[port.device]["tactile"],
        }
    )

print(
    json.dumps(
        {
            "p4_port": p4_port,
            "s3_port": s3_port,
            "ports": out_ports,
        },
        ensure_ascii=False,
    )
)
'@

$p4Override = if ($P4Port) { $P4Port.Trim() } else { "" }
$s3Override = if ($S3Port) { $S3Port.Trim() } else { "" }

$selectionJson = $DetectorCode | & $PythonCommand - $p4Override $s3Override
if ($LASTEXITCODE -ne 0) {
    throw "Serial port auto-detection failed."
}

$selection = $selectionJson | ConvertFrom-Json

Write-Host "Detected serial ports:"
foreach ($port in @($selection.ports)) {
    $roles = @()
    if ($port.device -eq $selection.p4_port) {
        $roles += "P4/control"
    }
    if ($port.device -eq $selection.s3_port) {
        $roles += "S3/tactile"
    }
    $roleText = ""
    if ($roles.Count -gt 0) {
        $roleText = " [" + ($roles -join " / ") + "]"
    }
    Write-Host ("  {0,-6} {1,-9} ctrl={2,-2} tac={3,-2} {4}{5}" -f `
        $port.device, $port.vidpid, $port.control_score, $port.tactile_score, $port.description, $roleText)
}

Write-Host ""
Write-Host "P4/control port : $($selection.p4_port)"
Write-Host "S3/tactile port : $($selection.s3_port)"
Write-Host ""

if ($DryRun) {
    Write-Host "Dry run only. GUI not started."
    exit 0
}

Push-Location $ScriptDir
try {
    & $PythonCommand $MainPy --app-mode control --mode monitor --port $selection.p4_port --tactile-port $selection.s3_port @ExtraArgs
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
