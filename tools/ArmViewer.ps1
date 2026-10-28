# ArmViewer.ps1 - live kinematic twin for the custom remote controller.
#
# Reads the chip2 (ArmController) debug stream on a COM port and draws a simple
# arm that follows the five potentiometers, so the controller pose can be seen
# without the real arm and without the main controller.
#
#   powershell -ExecutionPolicy Bypass -File tools\ArmViewer.ps1 -Port COM5
#   powershell -ExecutionPolicy Bypass -File tools\ArmViewer.ps1 -Replay build\potlog6.txt
#   powershell -ExecutionPolicy Bypass -File tools\ArmViewer.ps1 -SelfTest build\potlog6.txt
#
param(
    [string]$Port = 'COM5',
    [int]$Baud = 115200,
    [string]$Replay = '',
    [string]$SelfTest = '',
    [string]$Shot = '',
    [int]$AutoClose = 0,
    [switch]$Connect
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = 'Stop'

# Declare DPI awareness before any window is created, otherwise Windows stretches
# the whole (DPI-unaware) window and the drawing ends up blurry and partly off
# screen on scaled displays.
try {
    Add-Type -Namespace Win -Name Dpi -WarningAction SilentlyContinue -MemberDefinition '[DllImport("user32.dll")] public static extern bool SetProcessDPIAware();'
    [void][Win.Dpi]::SetProcessDPIAware()
} catch { }

# --- calibration ------------------------------------------------------------
# Raw-pot display coordinates for the kinematic preview, not actuator command
# angles. The firmware calibration lives in ArmController/UserApp/arm_targets.c;
# UART ANG reports those calibrated angles. Do not send these display values as POS.
# Fields: AdcMin, AdcMax, DegMin, DegMax, Dir  (degrees, not 0.01 deg)
$Cal = @(
    @{ Name = 'BASE';     AdcMin = 0; AdcMax = 4095; DegMin = 0; DegMax = 333; Dir = 1 },
    @{ Name = 'SHOULDER'; AdcMin = 0; AdcMax = 4095; DegMin = 0; DegMax = 333; Dir = 1 },
    @{ Name = 'ELBOW';    AdcMin = 0; AdcMax = 4095; DegMin = 0; DegMax = 333; Dir = 1 },
    @{ Name = 'WRISTP';   AdcMin = 0; AdcMax = 4095; DegMin = 0; DegMax = 333; Dir = 1 },
    @{ Name = 'WRISTR';   AdcMin = 0; AdcMax = 4095; DegMin = 0; DegMax = 333; Dir = 1 }
)

# Drawing trim: the calibrated angle is placeholder data until the arm exists, so
# the picture maps each pot over a plausible joint range. Span = degrees the joint
# travels over the whole pot, Zero slides it, Sign flips the joint when the pot
# runs backwards. All three are display trims and go back to the values below once
# $Cal carries real endpoints. Tune them live with 1-5 / arrows / S / 0.
$Draw = @(
    @{ Name = 'BASE';    Sign = -1; Span = 340; Zero = -170 },  # base yaw, pot runs backwards
    @{ Name = 'SHOULDER';Sign = -1; Span = 335; Zero =  193 },  # measured 2026-10-25
    @{ Name = 'ELBOW';   Sign =  1; Span = 315; Zero = -163 },  # relative bend
    @{ Name = 'WRISTP';  Sign = -1; Span = 300; Zero = -155 },  # relative pitch
    @{ Name = 'WRISTR';  Sign =  1; Span = 360; Zero = -180 }   # tool spin (not calibrated yet)
)
$DrawHome = @()
foreach ($r in $Draw) { $DrawHome += @{ Sign = $r.Sign; Span = $r.Span; Zero = $r.Zero } }
$script:TuneJoint = 1

# Link lengths in pixels for the side view.
$L1 = 165.0
$L2 = 145.0
$L3 = 70.0

function Convert-PotToDeg {
    param([int]$Adc, [hashtable]$C)
    if ($Adc -lt $C.AdcMin -or $Adc -gt $C.AdcMax) { return $null }
    $den = [double]($C.AdcMax - $C.AdcMin)
    if ($den -le 0) { return $null }
    $num = [double]($Adc - $C.AdcMin)
    if ($C.Dir -lt 0) { $num = $den - $num }
    return $C.DegMin + $num * ($C.DegMax - $C.DegMin) / $den
}

function Convert-DegToDraw {
    param([double]$Deg, [hashtable]$R)
    # placeholder span of $Cal is 0..333 deg
    $t = $Deg / 333.0
    if ($t -lt 0) { $t = 0 } elseif ($t -gt 1) { $t = 1 }
    return $R.Sign * ($R.Span * $t + [double]$R.Zero)
}

# --- sample model -----------------------------------------------------------
$script:State = @{
    Raw      = @(0, 0, 0, 0, 0)
    Deg      = @(0.0, 0.0, 0.0, 0.0, 0.0)
    Valid    = -1
    Pot      = -1
    Mode     = -1
    Keys     = '----'
    Ack      = -1
    Lines    = 0
    LastMs   = 0
    HaveData = $false
    Port     = $Port
    Error    = ''
}

$script:Filtered = $null

function Push-Line {
    param([string]$Line)

    $m = [regex]::Match($Line, 'POTRAW\s+(\d+),(\d+),(\d+),(\d+),(\d+)')
    if ($m.Success) {
        $raw = @()
        for ($i = 1; $i -le 5; $i++) { $raw += [int]$m.Groups[$i].Value }
        $script:State.Raw = $raw
        $deg = @()
        for ($i = 0; $i -lt 5; $i++) {
            $d = Convert-PotToDeg -Adc $raw[$i] -C $Cal[$i]
            if ($null -eq $d) { $d = 0.0 }
            $deg += [double]$d
        }
        $script:State.Deg = $deg
        $script:State.Lines++
        $script:State.LastMs = [Environment]::TickCount
        $script:State.HaveData = $true
        return
    }

    $m = [regex]::Match($Line, 'VALID=(\d).*KEYS=([0-9A-Fa-f]{4}).*ACK=(\d).*MODE=(\d).*POT=(\d)')
    if ($m.Success) {
        $script:State.Valid = [int]$m.Groups[1].Value
        $script:State.Keys = $m.Groups[2].Value.ToUpper()
        $script:State.Ack = [int]$m.Groups[3].Value
        $script:State.Mode = [int]$m.Groups[4].Value
        $script:State.Pot = [int]$m.Groups[5].Value
        return
    }
}

# --- replay / selftest ------------------------------------------------------
function Read-ReplayLines {
    param([string]$Path)
    $out = New-Object System.Collections.ArrayList
    foreach ($line in [System.IO.File]::ReadLines($Path)) {
        $t = 0
        $body = $line
        $m = [regex]::Match($line, '^(\d+)\s+(.*)$')
        if ($m.Success) { $t = [int]$m.Groups[1].Value; $body = $m.Groups[2].Value }
        [void]$out.Add(@{ t = $t; s = $body })
    }
    return $out
}

if ($SelfTest -ne '') {
    $items = Read-ReplayLines -Path $SelfTest
    foreach ($it in $items) { Push-Line -Line $it.s }
    Write-Output ('lines      = {0}' -f $items.Count)
    Write-Output ('potraw     = {0}' -f $script:State.Lines)
    Write-Output ('valid/pot  = {0}/{1}' -f $script:State.Valid, $script:State.Pot)
    Write-Output ('keys/mode  = {0}/{1}' -f $script:State.Keys, $script:State.Mode)
    for ($i = 0; $i -lt 5; $i++) {
        Write-Output ('{0,-9} adc={1,4}  deg={2,7:F2}' -f $Cal[$i].Name, $script:State.Raw[$i], $script:State.Deg[$i])
    }
    return
}

# --- GUI --------------------------------------------------------------------
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing

$C0 = [System.Drawing.Color]::FromArgb(24, 26, 30)
$script:Ink    = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(230, 232, 236))
$script:Dim    = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(130, 136, 146))
$script:Accent = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(90, 200, 255))
$script:Warn   = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(255, 170, 60))
$script:Tower  = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(60, 66, 76))
$script:Track  = New-Object System.Drawing.SolidBrush ([System.Drawing.Color]::FromArgb(45, 50, 58))
$script:JointPen  = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(90, 200, 255)), 3
$script:LinkPen   = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(210, 214, 220)), 3
$script:ThinPen   = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(70, 76, 86)), 1
$script:GridPen   = New-Object System.Drawing.Pen ([System.Drawing.Color]::FromArgb(45, 50, 58)), 1
$script:Font   = New-Object System.Drawing.Font 'Consolas', 9
$script:FontBig = New-Object System.Drawing.Font 'Consolas', 11, ([System.Drawing.FontStyle]::Bold)

if (-not ('BufferedPanel' -as [type])) {
    Add-Type -ReferencedAssemblies System.Windows.Forms, System.Drawing -TypeDefinition @'
using System.Windows.Forms;
public class BufferedPanel : Panel {
    public BufferedPanel() {
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.OptimizedDoubleBuffer |
                 ControlStyles.UserPaint | ControlStyles.ResizeRedraw, true);
        UpdateStyles();
    }
    protected override void OnPaintBackground(PaintEventArgs e) { }
}
'@
}

$form = New-Object System.Windows.Forms.Form
$form.Text = 'ARM TWIN - custom controller'
$form.ClientSize = New-Object System.Drawing.Size(980, 640)
$form.BackColor = $C0
$form.StartPosition = 'CenterScreen'
$form.KeyPreview = $true

$bar = New-Object System.Windows.Forms.Panel
$bar.Dock = 'Top'
$bar.Height = 34
$bar.BackColor = [System.Drawing.Color]::FromArgb(32, 35, 40)
$form.Controls.Add($bar)

$panel = New-Object BufferedPanel
$panel.Dock = 'Fill'
$panel.BackColor = $C0
$form.Controls.Add($panel)
$panel.BringToFront()

$lblPort = New-Object System.Windows.Forms.Label
$lblPort.Text = 'Port'
$lblPort.ForeColor = [System.Drawing.Color]::LightGray
$lblPort.AutoSize = $true
$lblPort.Location = New-Object System.Drawing.Point(8, 9)
$bar.Controls.Add($lblPort)

$cmbPort = New-Object System.Windows.Forms.ComboBox
$cmbPort.DropDownStyle = 'DropDown'
$cmbPort.Width = 90
$cmbPort.Location = New-Object System.Drawing.Point(40, 6)
foreach ($p in [System.IO.Ports.SerialPort]::GetPortNames() | Sort-Object) { [void]$cmbPort.Items.Add($p) }
if ($cmbPort.Items.Contains($Port)) { $cmbPort.SelectedItem = $Port } else { $cmbPort.Text = $Port }
$bar.Controls.Add($cmbPort)

$btnConnect = New-Object System.Windows.Forms.Button
$btnConnect.Text = 'Connect'
$btnConnect.Width = 84
$btnConnect.Location = New-Object System.Drawing.Point(138, 5)
$bar.Controls.Add($btnConnect)

$lblInfo = New-Object System.Windows.Forms.Label
$lblInfo.AutoSize = $true
$lblInfo.ForeColor = [System.Drawing.Color]::Gainsboro
$lblInfo.Location = New-Object System.Drawing.Point(232, 9)
$bar.Controls.Add($lblInfo)

$script:Serial = $null
$script:RxBuf = ''
$script:ReplayItems = $null
$script:ReplayIdx = 0
$script:ReplayClock = 0
$script:Canvas = $null

function Close-Serial {
    if ($null -ne $script:Serial) {
        try { $script:Serial.Close() } catch { }
        $script:Serial.Dispose()
        $script:Serial = $null
    }
}

function Open-Serial {
    param([string]$Name)
    Close-Serial
    try {
        $sp = New-Object System.IO.Ports.SerialPort $Name, $Baud, 'None', 8, 'One'
        $sp.ReadTimeout = 200
        $sp.DtrEnable = $false
        $sp.RtsEnable = $false
        $sp.Open()
        $script:Serial = $sp
        $script:RxBuf = ''
        $script:State.Error = ''
    } catch {
        $script:State.Error = $_.Exception.Message
    }
}

function Update-State {
    if ($null -ne $script:Serial) {
        try {
            $chunk = $script:Serial.ReadExisting()
            if ($chunk.Length -gt 0) {
                $script:RxBuf += $chunk
                while ($true) {
                    $i = $script:RxBuf.IndexOf("`n")
                    if ($i -lt 0) { break }
                    $line = $script:RxBuf.Substring(0, $i).Trim()
                    $script:RxBuf = $script:RxBuf.Substring($i + 1)
                    if ($line.Length -gt 0) { Push-Line -Line $line }
                }
                if ($script:RxBuf.Length -gt 4096) { $script:RxBuf = '' }
            }
        } catch {
            $script:State.Error = $_.Exception.Message
        }
    }
    elseif ($null -ne $script:ReplayItems) {
        $script:ReplayClock += 50
        $guard = 0
        while ($script:ReplayIdx -lt $script:ReplayItems.Count -and
               $script:ReplayItems[$script:ReplayIdx].t -le $script:ReplayClock -and $guard -lt 200) {
            Push-Line -Line $script:ReplayItems[$script:ReplayIdx].s
            $script:ReplayIdx++
            $guard++
        }
        if ($script:ReplayIdx -ge $script:ReplayItems.Count) {
            $script:ReplayIdx = 0
            $script:ReplayClock = 0
            $script:Filtered = $null
        }
    }
}

function Render {
    param($g, [int]$W, [int]$H)

    # target kinematics from the pot positions
    $q = @(0.0, 0.0, 0.0, 0.0, 0.0)
    for ($i = 0; $i -lt 5; $i++) {
        $q[$i] = Convert-DegToDraw -Deg $script:State.Deg[$i] -R $Draw[$i]
    }
    if ($null -eq $script:Filtered) { $script:Filtered = $q }
    else {
        $f = @(0.0, 0.0, 0.0, 0.0, 0.0)
        for ($i = 0; $i -lt 5; $i++) { $f[$i] = $script:Filtered[$i] + 0.6 * ($q[$i] - $script:Filtered[$i]) }
        $script:Filtered = $f
    }
    $f = $script:Filtered

    $sideW = [int]($W * 0.62)
    $rad = [Math]::PI / 180.0
    $l1 = 0.22 * $H
    $l2 = 0.19 * $H
    $l3 = 0.09 * $H

    # --- side view ---
    $ox = [double]($sideW * 0.36)
    $oy = [double]($H * 0.66)
    for ($gx = 20; $gx -lt $sideW - 20; $gx += 40) { $g.DrawLine($script:GridPen, $gx, 40, $gx, $H - 60) }
    $g.DrawLine($script:ThinPen, 20, [float]$oy, [float]($sideW - 20), [float]$oy)

    $p1 = $f[1] * $rad
    $p2 = $p1 + $f[2] * $rad
    $p3 = $p2 + $f[3] * $rad

    $x1 = $ox + $l1 * [Math]::Cos($p1); $y1 = $oy - $l1 * [Math]::Sin($p1)
    $x2 = $x1 + $l2 * [Math]::Cos($p2); $y2 = $y1 - $l2 * [Math]::Sin($p2)
    $x3 = $x2 + $l3 * [Math]::Cos($p3); $y3 = $y2 - $l3 * [Math]::Sin($p3)

    $g.FillRectangle($script:Tower, [float]($ox - 14), [float]($oy - 10), [float]28, [float]14)
    $g.DrawLine($script:LinkPen, [float]$ox, [float]$oy, [float]$x1, [float]$y1)
    $g.DrawLine($script:LinkPen, [float]$x1, [float]$y1, [float]$x2, [float]$y2)
    $g.DrawLine($script:LinkPen, [float]$x2, [float]$y2, [float]$x3, [float]$y3)

    foreach ($pt in @(@($x1, $y1), @($x2, $y2), @($x3, $y3))) {
        $g.FillEllipse($script:Accent, [float]($pt[0] - 5), [float]($pt[1] - 5), [float]10, [float]10)
    }

    # gripper jaws, fixed opening (the gripper has no potentiometer)
    $jx = $x3 + 26 * [Math]::Cos($p3); $jy = $y3 - 26 * [Math]::Sin($p3)
    $nx = -[Math]::Sin($p3); $ny = -[Math]::Cos($p3)
    $g.DrawLine($script:LinkPen, [float]$x3, [float]$y3, [float]$jx, [float]$jy)
    $g.DrawLine($script:ThinPen, [float]($jx + $nx * 14), [float]($jy + $ny * 14), [float]($jx + $nx * 6), [float]($jy + $ny * 6))
    $g.DrawLine($script:ThinPen, [float]($jx - $nx * 14), [float]($jy - $ny * 14), [float]($jx - $nx * 6), [float]($jy - $ny * 6))

    $g.DrawString('SIDE VIEW', $script:FontBig, $script:Dim, [float]20, [float]14)
    $g.DrawString(('shoulder {0,6:F1}   elbow {1,6:F1}   wrist {2,6:F1}' -f $f[1], $f[2], $f[3]), $script:Font, $script:Dim, [float]20, [float]38)

    # --- top view: base yaw ---
    $cx = [double]($sideW + ($W - $sideW) * 0.5)
    $cy = [double]($H * 0.30)
    $R = 86.0
    $g.DrawEllipse($script:ThinPen, [float]($cx - $R), [float]($cy - $R), [float](2 * $R), [float](2 * $R))
    $g.DrawEllipse($script:ThinPen, [float]($cx - $R * 0.62), [float]($cy - $R * 0.62), [float](2 * $R * 0.62), [float](2 * $R * 0.62))
    $g.DrawLine($script:GridPen, [float]($cx - $R), [float]$cy, [float]($cx + $R), [float]$cy)
    $g.DrawLine($script:GridPen, [float]$cx, [float]($cy - $R), [float]$cx, [float]($cy + $R))
    $yaw = $f[0] * $rad
    $tx = $cx + $R * [Math]::Sin($yaw); $ty = $cy - $R * [Math]::Cos($yaw)
    $g.DrawLine($script:JointPen, [float]$cx, [float]$cy, [float]$tx, [float]$ty)
    $g.DrawString(('TOP VIEW  base {0:F1}' -f $f[0]), $script:FontBig, $script:Dim, [float]($sideW + 16), [float]14)

    # --- readout ---
    $x0 = [float]($sideW + 24)
    $y = 330
    $g.DrawString('POT         ADC      DEG', $script:Font, $script:Dim, $x0, [float]($y - 22))
    for ($i = 0; $i -lt 5; $i++) {
        $g.DrawString(('{0,-10} {1,4}  {2,8:F1}' -f $Cal[$i].Name, $script:State.Raw[$i], $script:State.Deg[$i]), $script:Font, $script:Ink, $x0, [float]($y + $i * 20))
    }

    # --- status ---
    $st = 'WAITING'
    $brush = $script:Warn
    if ($script:State.Error -ne '') { $st = 'PORT ERROR: ' + $script:State.Error; $brush = $script:Warn }
    elseif ($script:State.HaveData) {
        $age = [Environment]::TickCount - $script:State.LastMs
        if ($age -gt 1500) { $st = 'STALE ' + [int]($age / 1000) + 's'; $brush = $script:Warn }
        else { $st = 'LIVE'; $brush = $script:Accent }
    }
    $modeTxt = @('LOCKED', 'RESUMING', 'ACTIVE')
    $mode = 'mode=?'
    if ($script:State.Mode -ge 0 -and $script:State.Mode -le 2) { $mode = 'mode=' + $modeTxt[$script:State.Mode] }
    $potTxt = 'pot=?'
    if ($script:State.Pot -eq 1) { $potTxt = 'pot=ON' } elseif ($script:State.Pot -eq 0) { $potTxt = 'pot=off' }
    $validTxt = 'valid=?'
    if ($script:State.Valid -eq 1) { $validTxt = 'valid=1' } elseif ($script:State.Valid -eq 0) { $validTxt = 'valid=0' }
    $ackTxt = 'ack=?'
    if ($script:State.Ack -eq 1) { $ackTxt = 'ack=1' } elseif ($script:State.Ack -eq 0) { $ackTxt = 'ack=0' }

    $line1 = '{0}   {1}   {2}   {3}   {4}   keys={5}' -f $st, $validTxt, $potTxt, $mode, $ackTxt, $script:State.Keys
    $g.DrawString($line1, $script:Font, $brush, [float]20, [float]($H - 52))
    $g.DrawString(('frames {0}   port {1}' -f $script:State.Lines, $script:State.Port), $script:Font, $script:Dim, [float]20, [float]($H - 30))

    $tj = $script:TuneJoint
    $tune = 'TUNE J{0} {1}  span {2}  zero {3}  sign {4}    [1-5] joint  [up/dn] zero  [lt/rt] span  [s] sign  [0] reset  [w] save' -f `
    ($tj + 1), $Draw[$tj].Name, $Draw[$tj].Span, $Draw[$tj].Zero, $(if ($Draw[$tj].Sign -lt 0) { '-1' } else { '+1' })
    $g.DrawString($tune, $script:Font, $script:Warn, [float]20, [float]($H - 74))
}

$panel.Add_Paint({
    param($sender, $e)
    $W = $panel.Width
    $H = $panel.Height
    if ($W -lt 32 -or $H -lt 32) { return }
    try {
        if ($null -eq $script:Canvas -or $script:Canvas.Width -ne $W -or $script:Canvas.Height -ne $H) {
            if ($null -ne $script:Canvas) { $script:Canvas.Dispose() }
            $script:Canvas = New-Object System.Drawing.Bitmap $W, $H
        }
        $g = [System.Drawing.Graphics]::FromImage($script:Canvas)
        $g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
        $g.Clear($C0)
        Render -g $g -W $W -H $H
        $g.Dispose()
        $e.Graphics.DrawImageUnscaled($script:Canvas, 0, 0)
    } catch {
        try {
            $e.Graphics.Clear($C0)
            $e.Graphics.DrawString('VIEW ERROR: ' + $_.Exception.Message, $script:Font, $script:Warn, [float]20, [float]60)
        } catch { }
    }
})

$timer = New-Object System.Windows.Forms.Timer
$timer.Interval = 33
$timer.Add_Tick({
    Update-State
    $modeTxt = @('LOCKED', 'RESUMING', 'ACTIVE')
    $m = '?'
    if ($script:State.Mode -ge 0 -and $script:State.Mode -le 2) { $m = $modeTxt[$script:State.Mode] }
    $lblInfo.Text = ('{0}  mode={1}  pot={2}  frames={3}' -f $script:State.Port, $m, $script:State.Pot, $script:State.Lines)
    $panel.Invalidate()
})

$btnConnect.Add_Click({
    if ($null -ne $script:Serial) {
        Close-Serial
        $btnConnect.Text = 'Connect'
    } else {
        $script:State.Port = $cmbPort.Text
        $script:State.HaveData = $false
        Open-Serial -Name $cmbPort.Text
        if ($null -ne $script:Serial) { $btnConnect.Text = 'Disconnect' }
        else { [void][System.Windows.Forms.MessageBox]::Show($script:State.Error) }
    }
})

$form.Add_KeyDown({
    param($sender, $e)
    $K = $e.KeyCode
    for ($i = 0; $i -lt 5; $i++) {
        if ($K -eq ([int][System.Windows.Forms.Keys]::D1 + $i)) {
            $script:TuneJoint = $i
            $e.Handled = $true
            $panel.Invalidate()
            return
        }
    }
    $j = $script:TuneJoint
    $fine = 5
    if ($e.Shift) { $fine = 1 }
    if ($K -eq [System.Windows.Forms.Keys]::Up) { $Draw[$j].Zero += $fine }
    elseif ($K -eq [System.Windows.Forms.Keys]::Down) { $Draw[$j].Zero -= $fine }
    elseif ($K -eq [System.Windows.Forms.Keys]::Right) { $Draw[$j].Span += $fine }
    elseif ($K -eq [System.Windows.Forms.Keys]::Left) { $Draw[$j].Span -= $fine }
    elseif ($K -eq [System.Windows.Forms.Keys]::S) { $Draw[$j].Sign = 0 - $Draw[$j].Sign }
    elseif ($K -eq [System.Windows.Forms.Keys]::D0) {
        $Draw[$j].Sign = $DrawHome[$j].Sign
        $Draw[$j].Span = $DrawHome[$j].Span
        $Draw[$j].Zero = $DrawHome[$j].Zero
    }
    elseif ($K -eq [System.Windows.Forms.Keys]::W) {
        $rows = @()
        foreach ($r in $Draw) { $rows += ('{0} sign={1} span={2} zero={3}' -f $r.Name, $r.Sign, $r.Span, $r.Zero) }
        [System.IO.File]::WriteAllLines((Join-Path $PSScriptRoot 'ArmViewer.tune.txt'), [string[]]$rows)
    }
    $e.Handled = $true
    $panel.Invalidate()
})

$form.Add_FormClosing({
    $timer.Stop()
    Close-Serial
    if ($null -ne $script:Canvas) { $script:Canvas.Dispose(); $script:Canvas = $null }
})

if ($Replay -ne '') {
    $script:ReplayItems = Read-ReplayLines -Path $Replay
    $script:ReplayIdx = 0
    $script:ReplayClock = 0
    $lblInfo.Text = 'replay: ' + $Replay
} else {
    Open-Serial -Name $Port
    if ($null -ne $script:Serial) { $btnConnect.Text = 'Disconnect' }
}

if ($Shot -ne '') {
    # Offline frame export: feed the whole recording (or nothing) and render one frame.
    $script:State.Error = ''
    if ($Replay -ne '') {
        foreach ($it in (Read-ReplayLines -Path $Replay)) { Push-Line -Line $it.s }
    }
    $sw = 980
    $sh = 600
    $sb = New-Object System.Drawing.Bitmap $sw, $sh
    $sg = [System.Drawing.Graphics]::FromImage($sb)
    $sg.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
    $sg.Clear($C0)
    Render -g $sg -W $sw -H $sh
    $sg.Dispose()
    $sb.Save($Shot, [System.Drawing.Imaging.ImageFormat]::Png)
    $sb.Dispose()
    Write-Output ('saved ' + $Shot)
    return
}

$timer.Start()

if ($Connect -and $Replay -eq '') {
    Open-Serial -Name $cmbPort.Text
    if ($null -ne $script:Serial) { $btnConnect.Text = 'Disconnect' }
}

if ($AutoClose -gt 0) {
    $t2 = New-Object System.Windows.Forms.Timer
    $t2.Interval = $AutoClose * 1000
    $t2.Add_Tick({ $t2.Stop(); $form.Close() })
    $t2.Start()
}

[void]$form.ShowDialog()
Close-Serial
