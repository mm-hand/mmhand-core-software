param(
    [string]$Label = "press",
    [string]$PortName = "COM11",
    [int]$Seconds = 10,
    [int]$DelaySeconds = 3
)

$port = New-Object System.IO.Ports.SerialPort $PortName, 115200, 'None', 8, 'One'
$port.ReadTimeout = 500
$max = @(0) * 15
$samples = 0

try {
    $port.Open()
    $port.DiscardInBuffer()
    if ($DelaySeconds -gt 0) {
        Start-Sleep -Seconds $DelaySeconds
    }

    $start = Get-Date
    while (((Get-Date) - $start).TotalSeconds -lt $Seconds) {
        try {
            $line = $port.ReadLine()
        } catch [System.TimeoutException] {
            continue
        }

        if ($line -notmatch '\[TactileMag\]') {
            continue
        }

        for ($g = 0; $g -lt 5; $g++) {
            if ($line -match ("G$g`:(\d+),(\d+),(\d+)")) {
                for ($s = 0; $s -lt 3; $s++) {
                    $value = [int]$Matches[$s + 1]
                    $index = $g * 3 + $s
                    if ($value -gt $max[$index]) {
                        $max[$index] = $value
                    }
                }
            }
        }
        $samples++
    }
} finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}

Write-Output "label=$Label samples=$samples"
for ($g = 0; $g -lt 5; $g++) {
    Write-Output ("G$g max: {0},{1},{2}" -f $max[$g * 3], $max[$g * 3 + 1], $max[$g * 3 + 2])
}

$sorted = @(
    for ($i = 0; $i -lt 15; $i++) {
        [pscustomobject]@{
            idx = $i
            g = [math]::Floor($i / 3)
            s = $i % 3
            v = $max[$i]
        }
    }
) | Sort-Object v -Descending | Select-Object -First 5

$sorted | ForEach-Object {
    Write-Output ("TOP G$($_.g)/S$($_.s) max=$($_.v)")
}
