import serial
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLViewWidget, GLLinePlotItem, GLMeshItem
import time
import os
import math

# --- Serial Port Configuration ---
serialPort = '/dev/ttyUSB0'
baudRate = 115200
timeout = 0.1
accMagMaxPoints = 1000
gyroMaxPoints = 100
trailMaxPoints = 100
gyroTimeWindow = 5.0
alpha = 0.3
epsilon = 1e-6

# --- Calibration Variables ---
gxOffset = 0.0
gyOffset = 0.0
gzOffset = 0.0
accCalibrating = False
magCalibrating = False
accMin = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
accMax = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
magMin = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
magMax = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
magRawMin = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
magRawMax = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
accOffset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
accScale = {'x': 1.0, 'y': 1.0, 'z': 1.0}
magOffset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
magScale = {'x': 1.0, 'y': 1.0, 'z': 1.0}
accFiltered = {'x': None, 'y': None, 'z': None}
magFiltered = {'x': None, 'y': None, 'z': None}
axSamples = []
aySamples = []
azSamples = []
mxSamples = []
mySamples = []
mzSamples = []
trailBuffer = []


# --- Data Storage ---
accRaw = {'x': np.empty(0), 'y': np.empty(0), 'z': np.empty(0)}
gyroRaw = {'x': np.empty(0), 'y': np.empty(0), 'z': np.empty(0)}
magRaw = {'x': np.empty(0), 'y': np.empty(0), 'z': np.empty(0)}
timePoints = np.empty(0)
startTime = None
ser = None
plotTimer = None

# --- Global UI References ---
plots = {}
scatterItems = {}
curveItems = {}
attitudeItems = {}
mag3DView = None
calibrateAccButton = None
calibrateMagButton = None

# --- Serial Connection Initialization ---
def init_serial():
    global ser
    try:
        ser = serial.Serial(serialPort, baudRate, timeout=timeout)
        ser.reset_input_buffer()
        return True
    except serial.SerialException as e:
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Serial Error")
        msg.setText(f"Failed to open {serialPort}: {str(e)}\nCheck: 1) Port, 2) Close Arduino IDE, 3) Permissions (dialout)")
        msg.exec_()
        return False

# --- Generate Arduino Code ---
def generateArduinoCode(to_file=False):
    code = [
        "void calibrateSensors() { ",
        "    const float axOffset =  " + str(accOffset['x']) +  "; ",
        "    const float ayOffset =  " + str(accOffset['y']) +  "; ",
        "    const float azOffset =  " + str(accOffset['z']) +  "; ",
        "    const float axScale =  " + str(accScale['x']) +  "; ",
        "    const float ayScale =  " + str(accScale['y']) +  "; ",
        "    const float azScale =  " + str(accScale['z']) +  "; ",
        "    const float gxOffset =  " + str(gxOffset) +  "; ",
        "    const float gyOffset =  " + str(gyOffset) +  "; ",
        "    const float gzOffset =  " + str(gzOffset) +  "; ",
        "    const float mxOffset =  " + str(magOffset['x']) +  "; ",
        "    const float myOffset =  " + str(magOffset['y']) +  "; ",
        "    const float mzOffset =  " + str(magOffset['z']) +  "; ",
        "    const float mxScale =  " + str(magScale['x']) +  "; ",
        "    const float myScale =  " + str(magScale['y']) +  "; ",
        "    const float mzScale =  " + str(magScale['z']) +  "; ",
        " ",
        "    AxCal = (AxRaw - axOffset) * axScale; ",
        "    AyCal = (AyRaw - ayOffset) * ayScale; ",
        "    AzCal = (AzRaw - azOffset) * azScale; ",
        "    GxCal = GxRaw * 180/PI - gxOffset; ",
        "    GyCal = GyRaw * 180/PI - gyOffset; ",
        "    GzCal = GzRaw * 180/PI - gzOffset; ",
        "    MxCal = (MxRaw - mxOffset) * mxScale; ",
        "    MyCal = (MyRaw - myOffset) * myScale; ",
        "    MzCal = (MzRaw - mzOffset) * mzScale; ",
        "} "
    ]
    if to_file:
        try:
            file_path = os.path.join(os.getcwd(), 'calData.txt')
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write("\n".join(code) + "\n")
            if os.path.exists(file_path) and os.path.getsize(file_path) > 0:
                return True
            raise Exception("File is empty")
        except Exception as e:
            msg = QtWidgets.QMessageBox()
            msg.setWindowTitle("File Write Error")
            msg.setText("Error writing calData.txt: " + str(e))
            msg.exec_()
            return False
    return True

# --- Load Calibration Files ---
def loadCalibration():
    global gxOffset, gyOffset, gzOffset, accOffset, accScale, magOffset, magScale, plots, mag3DView
    loaded = []
    try:
        if os.path.exists('gCal.txt'):
            with open('gCal.txt', 'r') as f:
                values = list(map(float, f.readline().strip().split(',')))
                if len(values) == 3: 
                    gxOffset, gyOffset, gzOffset = values
                    loaded.append(f"Gyro (°/s): Gx={gxOffset:.6f}, Gy={gyOffset:.6f}, Gz={gzOffset:.6f}")
        if os.path.exists('accCal.txt'):
            with open('accCal.txt', 'r') as f:
                values = list(map(float, f.readline().strip().split(',')))
                if len(values) == 6:
                    accOffset['x'], accScale['x'], accOffset['y'], accScale['y'], accOffset['z'], accScale['z'] = values
                    for key in ['accXY', 'accYZ', 'accXZ']:
                        if key in plots:
                            plots[key].setRange(xRange=(-1.2, 1.2), yRange=(-1.2, 1.2))
                    loaded.append(f"Acc (m/s²): Offsets x={accOffset['x']:.6f}, y={accOffset['y']:.6f}, z={accOffset['z']:.6f}, " +
                                  f"Scales x={accScale['x']:.6f}, y={accScale['y']:.6f}, z={accScale['z']:.6f}")
        if os.path.exists('magCal.txt'):
            with open('magCal.txt', 'r') as f:
                values = list(map(float, f.readline().strip().split(',')))
                if len(values) == 6:
                    magOffset['x'], magScale['x'], magOffset['y'], magScale['y'], magOffset['z'], magScale['z'] = values
                    for key in ['magXY', 'magYZ', 'magXZ']:
                        if key in plots:
                            plots[key].setRange(xRange=(-1.2, 1.2), yRange=(-1.2, 1.2))
                    if mag3DView:
                        mag3DView.opts['distance'] = 3
                        mag3DView.setCameraPosition(distance=3, elevation=30, azimuth=45)
                    loaded.append(f"Mag (µT): Offsets x={magOffset['x']:.6f}, y={magOffset['y']:.6f}, z={magOffset['z']:.6f}, " +
                                  f"Scales x={magScale['x']:.6f}, y={magScale['y']:.6f}, z={magScale['z']:.6f}")
        if loaded:
            msg = QtWidgets.QMessageBox()
            msg.setWindowTitle("Calibration Loaded")
            msg.setText("Loaded calibration:\n" + "\n".join(loaded))
            msg.exec_()
            generateArduinoCode()
        else:
            msg = QtWidgets.QMessageBox()
            msg.setWindowTitle("No Calibration Found")
            msg.setText("No calibration files found. Please calibrate the sensors.")
            msg.exec_()
    except Exception as e:
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Calibration Load Error")
        msg.setText(f"Error loading calibration files: {str(e)}")
        msg.exec_()

# --- Save Calibration ---
def saveCalibration():
    global gxOffset, gyOffset, gzOffset, accOffset, accScale, magOffset, magScale
    saved = []
    try:
        with open('gCal.txt', 'w') as f:
            f.write(f"{gxOffset},{gyOffset},{gzOffset}\n")
            saved.append(f"Gyro (°/s): Gx={gxOffset:.6f}, Gy={gyOffset:.6f}, Gz={gzOffset:.6f}")
        with open('accCal.txt', 'w') as f:
            f.write(f"{accOffset['x']},{accScale['x']},{accOffset['y']},{accScale['y']},{accOffset['z']},{accScale['z']}\n")
            saved.append(f"Acc (m/s²): Offsets x={accOffset['x']:.6f}, y={accOffset['y']:.6f}, z={accOffset['z']:.6f}, " +
                         f"Scales x={accScale['x']:.6f}, y={accScale['y']:.6f}, z={accScale['z']:.6f}")
        with open('magCal.txt', 'w') as f:
            f.write(f"{magOffset['x']},{magScale['x']},{magOffset['y']},{magScale['y']},{magOffset['z']},{magScale['z']}\n")
            saved.append(f"Mag (µT): Offsets x={magOffset['x']:.6f}, y={magOffset['y']:.6f}, z={magOffset['z']:.6f}, " +
                         f"Scales x={magScale['x']:.6f}, y={magScale['y']:.6f}, z={magScale['z']:.6f}")
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Calibration Saved")
        msg.setText("Saved calibration:\n" + "\n".join(saved))
        msg.exec_()
        generateArduinoCode()
    except Exception as e:
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Calibration Save Error")
        msg.setText(f"Error saving calibration files: {str(e)}")
        msg.exec_()

# --- Calibrate Gyroscope ---
def calibrateGyro():
    global gxOffset, gyOffset, gzOffset
    msg = QtWidgets.QMessageBox()
    msg.setWindowTitle("Calibrate Gyro")
    msg.setText("Keep the device still for calibration. Press OK to start.")
    msg.exec_()
    gxSamples = []
    gySamples = []
    gzSamples = []
    sampleCount = 0
    if ser:
        ser.reset_input_buffer()
    while sampleCount < 200:
        if ser and ser.in_waiting <= 0: 
            time.sleep(0.01)
            continue
        try:
            if ser:
                line = ser.readline().decode('utf-8').strip()
                values = line.split(',')
                if len(values) != 9: continue
                ax, ay, az, gx, gy, gz, mx, my, mz = map(float, values)
                #gx = gx * 180.0 / math.pi
                #gy = gy * 180.0 / math.pi
                #gz = gz * 180.0 / math.pi
                gxSamples.append(gx)
                gySamples.append(gy)
                gzSamples.append(gz)
                sampleCount += 1
        except Exception:
            pass
        time.sleep(0.01)
    if not gxSamples: return
    gxOffset = np.mean(gxSamples)
    gyOffset = np.mean(gySamples)
    gzOffset = np.mean(gzSamples)
    try:
        with open('gCal.txt', 'w') as f:
            f.write(f"{gxOffset},{gyOffset},{gzOffset}\n")
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Gyro Calibration Complete")
        msg.setText(f"Offsets (°/s): Gx={gxOffset:.6f}, Gy={gyOffset:.6f}, Gz={gzOffset:.6f}\nSaved to gCal.txt")
        msg.exec_()
        generateArduinoCode()
    except Exception as e:
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Gyro Calibration Error")
        msg.setText(f"Error saving gyro offsets: {str(e)}")
        msg.exec_()

# --- Calibrate Accelerometer ---
def calibrateAcc():
    global accCalibrating, accMin, accMax, accOffset, accScale, accRaw, axSamples, aySamples, azSamples, plots
    global calibrateAccButton
    if accCalibrating:
        if calibrateAccButton:
            calibrateAccButton.setText("Calibrate Accelerometer")
        accCalibrating = False
        if axSamples:
            accMin['x'] = min(axSamples)
            accMin['y'] = min(aySamples)
            accMin['z'] = min(azSamples)
            accMax['x'] = max(axSamples)
            accMax['y'] = max(aySamples)
            accMax['z'] = max(azSamples)
            accOffset['x'] = (accMin['x'] + accMax['x']) / 2.0
            accOffset['y'] = (accMin['y'] + accMax['y']) / 2.0
            accOffset['z'] = (accMin['z'] + accMax['z']) / 2.0
            rangeX = accMax['x'] - accMin['x']
            rangeY = accMax['y'] - accMin['y']
            rangeZ = accMax['z'] - accMin['z']
            avgRange = (rangeX + rangeY + rangeZ) / 3.0
            accScale['x'] = 2.0 / avgRange if avgRange != 0 else 1.0
            accScale['y'] = 2.0 / avgRange if avgRange != 0 else 1.0
            accScale['z'] = 2.0 / avgRange if avgRange != 0 else 1.0
            if avgRange < 0.5 or avgRange > 20.0:
                msg = QtWidgets.QMessageBox()
                msg.setWindowTitle("Calibration Warning")
                msg.setText(f"Average range {avgRange:.6f} is unusual.\nEnsure full 3D rotation during calibration.")
                msg.exec_()
            accRaw['x'] = np.empty(0)
            accRaw['y'] = np.empty(0)
            accRaw['z'] = np.empty(0)
            try:
                with open('accCal.txt', 'w') as f:
                    f.write(f"{accOffset['x']},{accScale['x']},{accOffset['y']},{accScale['y']},{accOffset['z']},{accScale['z']}\n")
                msg = QtWidgets.QMessageBox()
                msg.setWindowTitle("Acc Calibration Complete")
                msg.setText(f"Offsets (m/s²): Ax={accOffset['x']:.6f}, Ay={accOffset['y']:.6f}, Az={accOffset['z']:.6f}\n" +
                            f"Scales: Ax={accScale['x']:.6f}, Ay={accScale['y']:.6f}, Az={accScale['z']:.6f}\nSaved to accCal.txt")
                msg.exec_()
                generateArduinoCode()
            except Exception as e:
                msg = QtWidgets.QMessageBox()
                msg.setWindowTitle("Acc Calibration Error")
                msg.setText(f"Error saving acc offsets: {str(e)}")
                msg.exec_()
            for key in ['accXY', 'accYZ', 'accXZ']:
                if key in plots:
                    plots[key].setRange(xRange=(-1.2, 1.2), yRange=(-1.2, 1.2))
        return
    
    accCalibrating = True
    accMin = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
    accMax = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
    axSamples = []
    aySamples = []
    azSamples = []
    accRaw['x'] = np.empty(0)
    accRaw['y'] = np.empty(0)
    accRaw['z'] = np.empty(0)
    if calibrateAccButton:
        calibrateAccButton.setText("Stop Acc Calibration")
    msg = QtWidgets.QMessageBox()
    msg.setWindowTitle("Accelerometer Calibration")
    msg.setText("Rotate device in all directions (full 3D rotation) to capture min/max values for all axes.")
    msg.exec_()

# --- Calibrate Magnetometer ---
def calibrateMag():
    global magCalibrating, magMin, magMax, magOffset, magScale, magRaw, mxSamples, mySamples, mzSamples
    global magRawMin, magRawMax, plots, mag3DView, vectorArrow, vectorTrail, xyProjection, verticalComponent, trailBuffer
    global calibrateMagButton
    
    if magCalibrating:
        if calibrateMagButton:
            calibrateMagButton.setText("Calibrate Magnetometer")
        magCalibrating = False
        if mxSamples:
            magMin['x'] = min(mxSamples)
            magMin['y'] = min(mySamples)
            magMin['z'] = min(mzSamples)
            magMax['x'] = max(mxSamples)
            magMax['y'] = max(mySamples)
            magMax['z'] = max(mzSamples)
            
            magOffset['x'] = (magMin['x'] + magMax['x']) / 2.0
            magOffset['y'] = (magMin['y'] + magMax['y']) / 2.0
            magOffset['z'] = (magMin['z'] + magMax['z']) / 2.0
            
            rangeX = magMax['x'] - magMin['x']
            rangeY = magMax['y'] - magMin['y']
            rangeZ = magMax['z'] - magMin['z']
            
            magScale['x'] = 2.0 / rangeX if rangeX > 1e-6 else 1.0
            magScale['y'] = 2.0 / rangeY if rangeY > 1e-6 else 1.0
            magScale['z'] = 2.0 / rangeZ if rangeZ > 1e-6 else 1.0
            
            magRaw['x'] = np.empty(0)
            magRaw['y'] = np.empty(0)
            magRaw['z'] = np.empty(0)
            magRawMin = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
            magRawMax = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
            trailBuffer = []
            
            if vectorArrow:
                vectorArrow.setData(pos=np.array([[0, 0, 0], [0, 0, 0]]))
            if vectorTrail:
                vectorTrail.setData(pos=np.array([[0, 0, 0]]))
            if xyProjection:
                xyProjection.setData(pos=np.array([[0, 0, 0], [0, 0, 0]]))
            if verticalComponent:
                verticalComponent.setData(pos=np.array([[0, 0, 0], [0, 0, 0]]))
            
            try:
                with open('magCal.txt', 'w') as f:
                    f.write(f"{magOffset['x']},{magScale['x']},{magOffset['y']},{magScale['y']},{magOffset['z']},{magScale['z']}\n")
                msg = QtWidgets.QMessageBox()
                msg.setWindowTitle("Mag Calibration Complete")
                msg.setText(
                    f"Offsets (arb. units): Mx={magOffset['x']:.6f}, My={magOffset['y']:.6f}, Mz={magOffset['z']:.6f}\n"
                    f"Scales (per-axis to ±1): X={magScale['x']:.6f}, Y={magScale['y']:.6f}, Z={magScale['z']:.6f}\n"
                    f"Saved to magCal.txt\n"
                    f"Your XZ circle will now be perfectly round from -1 to +1"
                )
                msg.exec_()
                generateArduinoCode()
            except Exception as e:
                msg = QtWidgets.QMessageBox()
                msg.setWindowTitle("Mag Calibration Error")
                msg.setText(f"Error saving mag offsets: {str(e)}")
                msg.exec_()
            
            for key in ['magXY', 'magYZ', 'magXZ']:
                if key in plots:
                    plots[key].setRange(xRange=(-1.2, 1.2), yRange=(-1.2, 1.2))
            if mag3DView:
                mag3DView.opts['distance'] = 3
                mag3DView.setCameraPosition(distance=3, elevation=30, azimuth=45)
        return

    magCalibrating = True
    magMin = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
    magMax = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
    mxSamples = []
    mySamples = []
    mzSamples = []
    magRaw['x'] = np.empty(0)
    magRaw['y'] = np.empty(0)
    magRaw['z'] = np.empty(0)
    magRawMin = {'x': float('inf'), 'y': float('inf'), 'z': float('inf')}
    magRawMax = {'x': float('-inf'), 'y': float('-inf'), 'z': float('-inf')}
    trailBuffer = []
    
    if vectorArrow:
        vectorArrow.setData(pos=np.array([[0, 0, 0], [0, 0, 0]]))
    if vectorTrail:
        vectorTrail.setData(pos=np.array([[0, 0, 0]]))
    if xyProjection:
        xyProjection.setData(pos=np.array([[0, 0, 0], [0, 0, 0]]))
    if verticalComponent:
        verticalComponent.setData(pos=np.array([[0, 0, 0], [0, 0, 0]]))
    
    if calibrateMagButton:
        calibrateMagButton.setText("Stop Mag Calibration")
    msg = QtWidgets.QMessageBox()
    msg.setWindowTitle("Magnetometer Calibration")
    msg.setText("Rotate device in figure-8 pattern to capture min/max values for all axes.\n"
                "After calibration, XZ circle will be forced to ±1.0")
    msg.exec_()

def generateCode():
    success = generateArduinoCode(to_file=True)
    msg = QtWidgets.QMessageBox()
    msg.setWindowTitle("Generate Code")
    if success:
        msg.setText("Arduino calibration function saved to calData.txt")
    else:
        msg.setText("Error saving calData.txt.")
    msg.exec_()

# --- Stop Program ---
def stopProgram():
    try:
        if plotTimer:
            plotTimer.stop()
        if ser and ser.is_open:
            ser.close()
        if mainWindow:
            mainWindow.close()
        if mag3DWindow:
            mag3DWindow.close()
        app.exit(0)
    except Exception as e:
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Stop Error")
        msg.setText(f"Error stopping program: {str(e)}")
        msg.exec_()

# --- Update Plots ---
def updatePlots():
    global accRaw, gyroRaw, magRaw, timePoints, gxOffset, gyOffset, gzOffset
    global accCalibrating, magCalibrating, accMin, accMax, magMin, magMax
    global accOffset, accScale, magOffset, magScale, accFiltered, magFiltered
    global axSamples, aySamples, azSamples, mxSamples, mySamples, mzSamples
    global magRawMin, magRawMax, vectorArrow, vectorTrail, xyProjection, verticalComponent, trailBuffer
    global attitudeItems, attitudePlots
    
    if not ser or ser.in_waiting <= 0: return
    try:
        line = ser.readline().decode('utf-8').strip()
        values = line.split(',')
        if len(values) != 9: return
        ax, ay, az, gx, gy, gz, mx, my, mz = map(float, values)
        
        #gx = gx * 180.0 / math.pi
        #gy = gy * 180.0 / math.pi
        #gz = gz * 180.0 / math.pi
        
        accFiltered['x'] = alpha * ax + (1 - alpha) * accFiltered['x'] if accFiltered['x'] is not None else ax
        accFiltered['y'] = alpha * ay + (1 - alpha) * accFiltered['y'] if accFiltered['y'] is not None else ay 
        accFiltered['z'] = alpha * az + (1 - alpha) * accFiltered['z'] if accFiltered['z'] is not None else az
        ax = accFiltered['x']
        ay = accFiltered['y']
        az = accFiltered['z']
        
        magFiltered['x'] = alpha * mx + (1 - alpha) * magFiltered['x'] if magFiltered['x'] is not None else mx
        magFiltered['y'] = alpha * my + (1 - alpha) * magFiltered['y'] if magFiltered['y'] is not None else my
        magFiltered['z'] = alpha * mz + (1 - alpha) * magFiltered['z'] if magFiltered['z'] is not None else mz
        mx = magFiltered['x']
        my = magFiltered['y']
        mz = magFiltered['z']
        
        if accCalibrating:
            axSamples.append(ax)
            aySamples.append(ay)
            azSamples.append(az)
            accMin['x'] = min(accMin['x'], ax)
            accMin['y'] = min(accMin['y'], ay)
            accMin['z'] = min(accMin['z'], az)
            accMax['x'] = max(accMax['x'], ax)
            accMax['y'] = max(accMax['y'], ay)
            accMax['z'] = max(accMax['z'], az)
            
        if magCalibrating:
            mxSamples.append(mx)
            mySamples.append(my)
            mzSamples.append(mz)
            magMin['x'] = min(magMin['x'], mx)
            magMin['y'] = min(magMin['y'], my)
            magMin['z'] = min(magMin['z'], mz)
            magMax['x'] = max(magMax['x'], mx)
            magMax['y'] = max(magMax['y'], my)
            magMax['z'] = max(magMax['z'], mz)
            
        axCal = (ax - accOffset['x']) * accScale['x']
        ayCal = (ay - accOffset['y']) * accScale['y']
        azCal = (az - accOffset['z']) * accScale['z']
        gxCal = gx - gxOffset
        gyCal = gy - gyOffset
        gzCal = gz - gzOffset
        mxCal = (mx - magOffset['x']) * magScale['x']
        myCal = (my - magOffset['y']) * magScale['y']
        mzCal = (mz - magOffset['z']) * magScale['z']
        
        if accCalibrating:
            ax_plot = ax
            ay_plot = ay
            az_plot = az
        else:
            ax_plot = axCal
            ay_plot = ayCal
            az_plot = azCal
            
        if magCalibrating:
            mx_plot = mx
            my_plot = my
            mz_plot = mz
        else:
            mx_plot = mxCal
            my_plot = myCal
            mz_plot = mzCal
            
        roll = math.atan2(ayCal, azCal + epsilon) * 180 / math.pi
        pitch = math.atan2(-axCal, math.sqrt(ayCal**2 + azCal**2 + epsilon)) * 180 / math.pi
        phi = math.radians(roll)
        theta = math.radians(pitch)
        
        mxH = mxCal * math.cos(theta) + myCal * math.sin(phi) * math.sin(theta) + mzCal * math.cos(phi) * math.sin(theta)
        myH = myCal * math.cos(phi) - mzCal * math.sin(phi)
        yaw = math.atan2(myH, mxH + epsilon) * 180 / math.pi
        if yaw < 0:
            yaw += 360
            
        if 'rollPlot_readout' in attitudeItems:
            attitudeItems['rollPlot_readout'].setText(f"Roll: {-roll:+.1f}°")
        if 'pitchPlot_readout' in attitudeItems:
            attitudeItems['pitchPlot_readout'].setText(f"Pitch: {-pitch:+.1f}°")
        if 'yawPlot_readout' in attitudeItems:
            attitudeItems['yawPlot_readout'].setText(f"Yaw: {yaw:.1f}°")
            
        vec = np.array([mx_plot, my_plot, mz_plot])
        norm = np.linalg.norm(vec)
        if norm > epsilon:
            unitVec = vec / norm
            if vectorArrow:
                vectorArrow.setData(pos=np.array([[0, 0, 0], unitVec]), color=(1, 0, 0, 1), width=3)
            xyProj = np.array([unitVec[0], unitVec[1], 0])
            if xyProjection:
                xyProjection.setData(pos=np.array([[0, 0, 0], xyProj]), color=(0, 1, 1, 1), width=2)
            if verticalComponent:
                verticalComponent.setData(pos=np.array([xyProj, unitVec]), color=(1, 0, 1, 1), width=2)
            trailBuffer.append(unitVec)
            trailBuffer = trailBuffer[-trailMaxPoints:]
            if vectorTrail:
                vectorTrail.setData(pos=np.array(trailBuffer), color=(1, 1, 0, 0.6), width=1)
                
        if accCalibrating or (accScale['x'] == 1.0 and accOffset['x'] == 0.0):
            accRawMin = {'x': min(accRaw['x'], default=ax_plot) if len(accRaw['x']) > 0 else ax_plot,
                         'y': min(accRaw['y'], default=ay_plot) if len(accRaw['y']) > 0 else ay_plot,
                         'z': min(accRaw['z'], default=az_plot) if len(accRaw['z']) > 0 else az_plot}
            accRawMax = {'x': max(accRaw['x'], default=ax_plot) if len(accRaw['x']) > 0 else ax_plot,
                         'y': max(accRaw['y'], default=ay_plot) if len(accRaw['y']) > 0 else ay_plot,
                         'z': max(accRaw['z'], default=az_plot) if len(accRaw['z']) > 0 else az_plot}
            xMin = accRawMin['x'] * 1.1
            xMax = accRawMax['x'] * 1.1
            yMin = accRawMin['y'] * 1.1
            yMax = accRawMax['y'] * 1.1
            zMin = accRawMin['z'] * 1.1
            zMax = accRawMax['z'] * 1.1
            if 'accXY' in plots:
                plots['accXY'].setRange(xRange=(xMin, xMax), yRange=(yMin, yMax))
            if 'accYZ' in plots:
                plots['accYZ'].setRange(xRange=(yMin, yMax), yRange=(zMin, zMax))
            if 'accXZ' in plots:
                plots['accXZ'].setRange(xRange=(xMin, xMax), yRange=(zMin, zMax))
                
        use_raw_mag_range = magCalibrating or (magScale['x'] == 1.0 and magOffset['x'] == 0.0)
        if use_raw_mag_range:
            magRawMin['x'] = min(magRawMin['x'], mx)
            magRawMax['x'] = max(magRawMax['x'], mx)
            magRawMin['y'] = min(magRawMin['y'], my)
            magRawMax['y'] = max(magRawMax['y'], my)
            magRawMin['z'] = min(magRawMin['z'], mz)
            magRawMax['z'] = max(magRawMax['z'], mz)
            xMin = magRawMin['x'] * 1.1
            xMax = magRawMax['x'] * 1.1
            yMin = magRawMin['y'] * 1.1
            yMax = magRawMax['y'] * 1.1
            zMin = magRawMin['z'] * 1.1
            zMax = magRawMax['z'] * 1.1
            if mag3DView:
                mag3DView.opts['distance'] = max(xMax - xMin, yMax - yMin, zMax - zMin) * 1.5
                mag3DView.setCameraPosition(distance=mag3DView.opts['distance'])
            if 'magXY' in plots:
                plots['magXY'].setRange(xRange=(xMin, xMax), yRange=(yMin, yMax))
            if 'magYZ' in plots:
                plots['magYZ'].setRange(xRange=(yMin, yMax), yRange=(zMin, zMax))
            if 'magXZ' in plots:
                plots['magXZ'].setRange(xRange=(xMin, xMax), yRange=(zMin, zMax))
                
        accRaw['x'] = np.append(accRaw['x'], ax_plot)[-accMagMaxPoints:]
        accRaw['y'] = np.append(accRaw['y'], ay_plot)[-accMagMaxPoints:]
        accRaw['z'] = np.append(accRaw['z'], az_plot)[-accMagMaxPoints:]
        gyroRaw['x'] = np.append(gyroRaw['x'], gxCal)[-gyroMaxPoints:]
        gyroRaw['y'] = np.append(gyroRaw['y'], gyCal)[-gyroMaxPoints:]
        gyroRaw['z'] = np.append(gyroRaw['z'], gzCal)[-gyroMaxPoints:]
        magRaw['x'] = np.append(magRaw['x'], mx_plot)[-accMagMaxPoints:]
        magRaw['y'] = np.append(magRaw['y'], my_plot)[-accMagMaxPoints:]
        magRaw['z'] = np.append(magRaw['z'], mz_plot)[-accMagMaxPoints:]
        
        if startTime:
            elapsed = startTime.msecsTo(QtCore.QTime.currentTime()) / 1000.0
            timePoints = np.append(timePoints, elapsed)[-gyroMaxPoints:]
            
        if 'accXY' in scatterItems:
            scatterItems['accXY'].setData(x=accRaw['x'], y=accRaw['y'])
        if 'accYZ' in scatterItems:
            scatterItems['accYZ'].setData(x=accRaw['y'], y=accRaw['z'])
        if 'accXZ' in scatterItems:
            scatterItems['accXZ'].setData(x=accRaw['x'], y=accRaw['z'])
        if 'magXY' in scatterItems:
            scatterItems['magXY'].setData(x=magRaw['x'], y=magRaw['y'])
        if 'magYZ' in scatterItems:
            scatterItems['magYZ'].setData(x=magRaw['y'], y=magRaw['z'])
        if 'magXZ' in scatterItems:
            scatterItems['magXZ'].setData(x=magRaw['x'], y=magRaw['z'])
            
        if len(timePoints) <= 0: return
        xMin = max(0, timePoints[-1] - gyroTimeWindow)
        xMax = timePoints[-1]
        for key in ['gyroX', 'gyroY', 'gyroZ']:
            if key in plots:
                plots[key].setRange(xRange=(xMin, xMax))
            if key in curveItems:
                curveItems[key].setData(x=timePoints, y=gyroRaw[key[-1].lower()])
                
        if 'roll_horizon' in attitudeItems:
            rollRad = math.radians(roll)
            x1 = -1 * math.cos(rollRad)
            y1 = 1 * math.sin(rollRad)
            x2 = 1 * math.cos(rollRad)
            y2 = -1 * math.sin(rollRad)
            attitudeItems['roll_horizon'].setData(x=[x1, x2], y=[y1, y2])
            
        if 'pitch_ladder' in attitudeItems:
            pitchOffset = -pitch / 90 * 1.2
            for idx, angle in enumerate([-90, -60, -30, -10, 0, 10, 30, 60, 90]):
                yBase = angle / 90 * 1.2
                yShifted = yBase + pitchOffset
                if angle == 0:
                    attitudeItems['pitch_ladder'][idx].setData(x=[-1, 1], y=[yShifted, yShifted])
                else:
                    attitudeItems['pitch_ladder'][idx].setData(x=[-0.5, -0.2, -0.2, 0.2, 0.2, 0.5], y=[yShifted, yShifted, yShifted+0.05, yShifted+0.05, yShifted, yShifted])
                    
        if 'yaw_needle' in attitudeItems:
            yawRad = math.radians(yaw)
            needleX = 1 * math.sin(yawRad)
            needleY = 1 * math.cos(yawRad)
            attitudeItems['yaw_needle'].setData(x=[0, needleX], y=[0, needleY])
    except Exception:
        pass

# --- MAIN EXECUTION ---
if __name__ == "__main__":
    # 1. Сначала создаем QApplication
    app = QtWidgets.QApplication([])
    
    # 2. Инициализируем время
    startTime = QtCore.QTime.currentTime()
    
    # 3. Инициализируем сериальный порт
    if not init_serial():
        exit(1)
    
    # 4. Создаем главное окно
    mainWindow = QtWidgets.QWidget()
    mainWindow.setWindowTitle("9-Axis IMU Visualization (Acc/Mag: 1000 pts, Gyro: 100 pts)")
    mainWindow.setMinimumSize(1000, 800)
    mainLayout = QtWidgets.QVBoxLayout()
    mainWindow.setLayout(mainLayout)
    
    plotGrid = QtWidgets.QGridLayout()
    plotTitles = [
        ('accXY', 'Acc XY (m/s²)', 'X', 'Y', (255, 0, 0, 255)),
        ('accYZ', 'Acc YZ (m/s²)', 'Y', 'Z', (0, 255, 0, 255)),
        ('accXZ', 'Acc XZ (m/s²)', 'X', 'Z', (0, 0, 255, 255)),
        ('magXY', 'Mag XY (µT)', 'X', 'Y', (0, 255, 255, 255)),
        ('magYZ', 'Mag YZ (µT)', 'Y', 'Z', (255, 0, 255, 255)),
        ('magXZ', 'Mag XZ (µT)', 'X', 'Z', (255, 255, 0, 255)),
        ('gyroX', 'Gyro X (°/s)', 'Time (s)', 'X', None),
        ('gyroY', 'Gyro Y (°/s)', 'Time (s)', 'Y', None),
        ('gyroZ', 'Gyro Z (°/s)', 'Time (s)', 'Z', None)
    ]
    
    for idx, (key, title, xLabel, yLabel, color) in enumerate(plotTitles):
        plot = pg.PlotWidget(title=title)
        plot.setMinimumSize(200, 200)
        if 'acc' in key or 'mag' in key:
            plot.setAspectLocked(True)
            scatterItems[key] = pg.ScatterPlotItem(size=5, brush=pg.mkBrush(*color), pen=None)
            plot.addItem(scatterItems[key])
            if 'acc' in key:
                plot.setRange(xRange=(-10, 10), yRange=(-10, 10))
            if 'mag' in key:
                plot.setRange(xRange=(-1.2, 1.2), yRange=(-1.2, 1.2))
        elif 'gyro' in key:
            plot.setRange(yRange=(-5, 5))
        
        plot.showGrid(x=True, y=True)
        plot.setLabel('bottom', xLabel)
        plot.setLabel('left', yLabel)
        
        if 'gyro' in key:
            curveItems[key] = plot.plot(pen=pg.mkPen('b', width=2))
            
        plotGrid.addWidget(plot, idx // 3, idx % 3)
        plots[key] = plot
    
    mainLayout.addLayout(plotGrid)
    
    buttonLayout = QtWidgets.QHBoxLayout()
    calibrateGyroButton = QtWidgets.QPushButton("Calibrate Gyro")
    calibrateGyroButton.setMinimumSize(200, 50)
    calibrateGyroButton.setStyleSheet("background-color: #00FF00; color: black; font-size: 16px;")
    
    calibrateAccButton = QtWidgets.QPushButton("Calibrate Accelerometer")
    calibrateAccButton.setMinimumSize(200, 50)
    calibrateAccButton.setStyleSheet("background-color: #FFFF00; color: black; font-size: 16px;")
    
    calibrateMagButton = QtWidgets.QPushButton("Calibrate Magnetometer")
    calibrateMagButton.setMinimumSize(200, 50)
    calibrateMagButton.setStyleSheet("background-color: #FFFF00; color: black; font-size: 16px;")
    
    saveCalButton = QtWidgets.QPushButton("Save Calibration")
    saveCalButton.setMinimumSize(200, 50)
    saveCalButton.setStyleSheet("background-color: #0000FF; color: white; font-size: 16px;")
    
    generateCodeButton = QtWidgets.QPushButton("Generate Code")
    generateCodeButton.setMinimumSize(200, 50)
    generateCodeButton.setStyleSheet("background-color: #0000FF; color: white; font-size: 16px;")
    
    stopButton = QtWidgets.QPushButton("Stop")
    stopButton.setMinimumSize(200, 50)
    stopButton.setStyleSheet("background-color: #FF0000; color: white; font-size: 16px;")
    
    buttonLayout.addWidget(calibrateGyroButton)
    buttonLayout.addWidget(calibrateAccButton)
    buttonLayout.addWidget(calibrateMagButton)
    buttonLayout.addWidget(saveCalButton)
    buttonLayout.addWidget(generateCodeButton)
    buttonLayout.addWidget(stopButton)
    mainLayout.addLayout(buttonLayout)
    
    # 5. Создаем 3D окно
    mag3DWindow = QtWidgets.QWidget()
    mag3DWindow.setWindowTitle("3D Magnetic Field Vector and Attitude (µT, °)")
    mag3DWindow.setMinimumSize(800, 800)
    mag3DLayout = QtWidgets.QGridLayout()
    mag3DWindow.setLayout(mag3DLayout)
    
    mag3DView = GLViewWidget()
    mag3DView.setCameraPosition(distance=3, elevation=30, azimuth=45)
    mag3DLayout.addWidget(mag3DView, 0, 0)
    
    sphereMesh = gl.MeshData.sphere(rows=20, cols=20, radius=1.0)
    sphere = gl.GLMeshItem(meshdata=sphereMesh, smooth=True, color=(0.3, 0.3, 0.3, 0.2), shader='shaded', drawFaces=True)
    mag3DView.addItem(sphere)
    
    circleMesh = gl.MeshData.cylinder(rows=1, cols=50, radius=[1.0, 1.0], length=0.001)
    xyCircle = gl.GLMeshItem(meshdata=circleMesh, smooth=True, color=(1, 0.5, 0, 0.4), shader='shaded', glOptions='additive')
    xyCircle.rotate(90, 1, 0, 0)
    xyCircle.setDepthValue(-5)
    mag3DView.addItem(xyCircle)
    
    for axis, color in zip([(1.2, 0, 0), (0, 1.2, 0), (0, 0, 1.2)], [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)]):
        line = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], axis]), color=color, width=2, antialias=True)
        mag3DView.addItem(line)
    
    vectorArrow = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 0]]), color=(1, 0, 0, 1), width=3, antialias=True)
    vectorArrow.setDepthValue(10)
    mag3DView.addItem(vectorArrow)
    
    xyProjection = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 0]]), color=(0, 1, 1, 1), width=2, antialias=True)
    xyProjection.setDepthValue(10)
    mag3DView.addItem(xyProjection)
    
    verticalComponent = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 0]]), color=(1, 0, 1, 1), width=2, antialias=True)
    verticalComponent.setDepthValue(10)
    mag3DView.addItem(verticalComponent)
    
    vectorTrail = gl.GLLinePlotItem(pos=np.array([[0, 0, 0]]), color=(1, 1, 0, 0.6), width=1, antialias=True)
    vectorTrail.setDepthValue(10)
    mag3DView.addItem(vectorTrail)
    
    attitudePlots = {}
    attitudePlotTitles = [
        ('rollPlot', 'Roll (°)', '', ''),
        ('pitchPlot', 'Pitch (°)', '', ''),
        ('yawPlot', 'Yaw (°)', '', '')
    ]
    
    for idx, (key, title, xLabel, yLabel) in enumerate(attitudePlotTitles):
        plot = pg.PlotWidget(title=title)
        plot.setMinimumSize(200, 200)
        plot.setRange(xRange=(-1.5, 1.5), yRange=(-1.5, 1.5))
        plot.setAspectLocked(True)
        plot.hideAxis('bottom')
        plot.hideAxis('left')
        readout = pg.TextItem("0.0°", anchor=(0.5, 0.5), color=(0, 255, 0))
        readout.setFont(QtGui.QFont("Courier New", 14, QtGui.QFont.Bold))
        if key in ['rollPlot', 'pitchPlot']:
            readout.setPos(0, -1.4)
        else:
            readout.setPos(0, -1.5)
        readout.setZValue(15)
        plot.addItem(readout)
        attitudeItems[f'{key}_readout'] = readout
        
        if key == 'rollPlot':
            skyX = np.array([-1.5, 1.5, 1.5, -1.5])
            skyY = np.array([0, 0, 1.5, 1.5])
            groundX = np.array([-1.5, 1.5, 1.5, -1.5])
            groundY = np.array([-1.5, -1.5, 0, 0])
            sky = pg.PlotDataItem(x=skyX, y=skyY, pen=None, fillLevel=1.5, brush=pg.mkBrush(0, 191, 255, 255))
            ground = pg.PlotDataItem(x=groundX, y=groundY, pen=None, fillLevel=-1.5, brush=pg.mkBrush(139, 69, 19, 255))
            plot.addItem(sky)
            plot.addItem(ground)
            horizon = pg.PlotDataItem(x=[-1, 1], y=[0, 0], pen=pg.mkPen(color=(0, 100, 0), width=3))
            plot.addItem(horizon)
            attitudeItems['roll_horizon'] = horizon
            for xPos, anchor in [(-1.2, (1, 0.5)), (1.2, (0, 0.5))]:
                text = pg.TextItem("0°", anchor=anchor, color=(255, 255, 0))
                text.setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))
                text.setPos(xPos, 0)
                text.setZValue(10)
                plot.addItem(text)
            for angle in [30, 60]:
                rad = math.radians(angle)
                sinRad = math.sin(rad)
                cosRad = math.cos(rad)
                text = pg.TextItem(f"{angle}°", anchor=(0, 0.5), color=(255, 255, 0))
                text.setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))
                text.setPos(1.2 * cosRad, 1.2 * sinRad)
                text.setZValue(10)
                plot.addItem(text)
                text = pg.TextItem(f"{angle}°", anchor=(1, 0.5), color=(255, 255, 0))
                text.setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))
                text.setPos(-1.2 * cosRad, 1.2 * sinRad)
                text.setZValue(10)
                plot.addItem(text)
                text = pg.TextItem(f"-{angle}°", anchor=(1, 0.5), color=(255, 255, 0))
                text.setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))
                text.setPos(-1.2 * cosRad, -1.2 * sinRad)
                text.setZValue(10)
                plot.addItem(text)
                text = pg.TextItem(f"-{angle}°", anchor=(0, 0.5), color=(255, 255, 0))
                text.setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))
                text.setPos(1.2 * cosRad, -1.2 * sinRad)
                text.setZValue(10)
                plot.addItem(text)
        elif key == 'pitchPlot':
            skyX = np.array([-1.5, 1.5, 1.5, -1.5])
            skyY = np.array([0, 0, 1.5, 1.5])
            groundX = np.array([-1.5, 1.5, 1.5, -1.5])
            groundY = np.array([-1.5, -1.5, 0, 0])
            sky = pg.PlotDataItem(x=skyX, y=skyY, pen=None, fillLevel=1.5, brush=pg.mkBrush(0, 191, 255, 255))
            ground = pg.PlotDataItem(x=groundX, y=groundY, pen=None, fillLevel=-1.5, brush=pg.mkBrush(139, 69, 19, 255))
            plot.addItem(sky)
            plot.addItem(ground)
            ladder = []
            for angle in [-90, -60, -30, -10, 0, 10, 30, 60, 90]:
                y = angle / 90 * 1.2
                if angle == 0:
                    line = pg.PlotDataItem(x=[-1, 1], y=[y, y], pen=pg.mkPen(color=(0, 100, 0), width=3)) 
                else:
                    line = pg.PlotDataItem(x=[-0.5, -0.2, -0.2, 0.2, 0.2, 0.5], y=[y, y, y+0.05, y+0.05, y, y], pen=pg.mkPen(color=(0, 100, 0), width=2))
                textLeft = pg.TextItem(f"{angle}°", anchor=(1, 0.5), color=(255, 255, 0))
                textLeft.setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))
                textLeft.setPos(-0.6, y)
                textRight = pg.TextItem(f"{angle}°", anchor=(0, 0.5), color=(255, 255, 0))
                textRight.setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))
                textRight.setPos(0.6, y)
                plot.addItem(textLeft)
                plot.addItem(textRight)
                plot.addItem(line)
                ladder.append(line)
            attitudeItems['pitch_ladder'] = ladder
        elif key == 'yawPlot':
            theta = np.linspace(0, 2 * np.pi, 100)
            xCircle = 1.2 * np.sin(theta)
            yCircle = 1.2 * np.cos(theta)
            circle = pg.PlotDataItem(x=xCircle, y=yCircle, pen=pg.mkPen('w', width=2), fillLevel=0, brush=pg.mkBrush(0, 0, 0, 255))
            plot.addItem(circle)
            for angle, label in [(0, 'N'), (90, 'E'), (180, 'S'), (270, 'W')]:
                rad = math.radians(angle)
                x = 1.3 * math.sin(rad)
                y = 1.3 * math.cos(rad)
                text = pg.TextItem(label, anchor=(0.5, 0.5), color=(255, 255, 0))
                text.setFont(QtGui.QFont("Arial", 12))
                text.setPos(x, y)
                plot.addItem(text)
            needle = pg.PlotDataItem(x=[0, 0], y=[0, 1], pen=pg.mkPen('r', width=3))
            plot.addItem(needle)
            attitudeItems['yaw_needle'] = needle
            
        if key == 'rollPlot':
            mag3DLayout.addWidget(plot, 0, 1)
        elif key == 'pitchPlot':
            mag3DLayout.addWidget(plot, 1, 0)
        elif key == 'yawPlot':
            mag3DLayout.addWidget(plot, 1, 1)
        attitudePlots[key] = plot
    
    # 6. Подключаем кнопки
    calibrateGyroButton.clicked.connect(calibrateGyro)
    calibrateAccButton.clicked.connect(calibrateAcc)
    calibrateMagButton.clicked.connect(calibrateMag)
    saveCalButton.clicked.connect(saveCalibration)
    generateCodeButton.clicked.connect(generateCode)
    stopButton.clicked.connect(stopProgram)
    
    ser.reset_input_buffer()
    loadCalibration()
    
    plotTimer = QtCore.QTimer()
    plotTimer.timeout.connect(updatePlots)
    plotTimer.start(50)
    
    mainWindow.show()
    mag3DWindow.show()
    
    try:
        app.exec_()
    except Exception as e:
        msg = QtWidgets.QMessageBox()
        msg.setWindowTitle("Application Error")
        msg.setText(f"Application error: {str(e)}")
        msg.exec_()
    finally:
        if ser and ser.is_open:
            ser.close()
