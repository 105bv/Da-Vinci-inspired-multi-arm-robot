# DaVinci Computer Vision

Computer vision subsystem for the DaVinci manufacturing project.

The current system uses a custom **YOLO26n** object detector to identify and track:

- `defective`
- `nondefective`

For each detected object, the vision pipeline provides:

```text
class
confidence
bounding box
center (cx, cy)
tracking ID
```

The main inference program is:

```text
vision/yolo_detect.py
```

Detection and ByteTrack tracking are integrated into the same pipeline.

---

# 1. Setup

Run the following commands from:

```bash
software/computer_vision
```

For example:

```bash
cd ~/davinci_ws/src/Da-Vinci-inspired-multi-arm-robot/software/computer_vision
```

Create the Python environment:

```bash
python3 -m venv .venv
```

Activate it:

```bash
source .venv/bin/activate
```

Install the required packages:

```bash
pip install -r requirements.txt
```

Current direct dependencies are:

```text
Ultralytics
OpenCV
PyYAML
```

When the environment is active, the terminal should begin with:

```text
(.venv)
```

---

# 2. Prepare Images

Collect images containing the objects that the model should detect.

The current classes are:

```text
0 = defective
1 = nondefective
```

The dataset should include variation in:

- object position
- object rotation
- distance from camera
- lighting
- shadows
- background
- multiple objects
- empty/background scenes when useful

Avoid using large numbers of nearly identical consecutive video frames.

If nearly identical frames are randomly divided between training and validation, validation results can appear better than the model actually performs on new scenes.

---

# 3. Label Images with Label Studio

Label Studio can be installed in a separate environment so it does not interfere with the computer vision environment.

From:

```bash
software/computer_vision
```

create the Label Studio environment:

```bash
python3 -m venv .labelstudio-venv
```

Activate it:

```bash
source .labelstudio-venv/bin/activate
```

Install Label Studio:

```bash
pip install label-studio
```

Start Label Studio:

```bash
label-studio
```

Label Studio should open in a web browser.

Create a new project using an **Object Detection / Bounding Boxes** labeling configuration.

Create these labels:

```text
defective
nondefective
```

Import the training images and draw a tight bounding box around every relevant object.

If an image contains multiple relevant objects, label all of them.

When labeling is finished, export the project in **YOLO format**.

After finishing with Label Studio, the environment can be closed with:

```bash
deactivate
```

Then reactivate the computer vision environment:

```bash
source .venv/bin/activate
```

---

# 4. Export the Dataset into `custom_data/`

The YOLO export from Label Studio should contain files similar to:

```text
images/
labels/
classes.txt
```

Assume the downloaded export is:

```text
~/Downloads/project.zip
```

Remove an older exported dataset if replacing it:

```bash
rm -rf custom_data/images
rm -rf custom_data/labels
rm -f custom_data/classes.txt
rm -f custom_data/notes.json
```

Then unzip the new export:

```bash
unzip ~/Downloads/project.zip -d custom_data
```

Check the result:

```bash
tree custom_data -L 2
```

The expected structure is:

```text
custom_data/
├── images/
│   ├── image001.jpg
│   ├── image002.jpg
│   └── ...
│
├── labels/
│   ├── image001.txt
│   ├── image002.txt
│   └── ...
│
└── classes.txt
```

`classes.txt` should contain:

```text
defective
nondefective
```

YOLO label files use this format:

```text
class_id x_center y_center width height
```

For example:

```text
0 0.484 0.517 0.218 0.286
```

The bounding-box coordinates are normalized between `0` and `1`.

---

# 5. Create the Training / Validation Split

The included script creates an 80/20 training-validation split.

Run:

```bash
python scripts/train_val_split.py \
    --datapath="custom_data" \
    --train_pct=.8
```

This creates:

```text
data/
├── train/
│   ├── images/
│   └── labels/
│
└── validation/
    ├── images/
    └── labels/
```

## Important: Re-splitting the Dataset

The split script does **not automatically remove files from an older split**.

If `custom_data/` changes, delete the old `data/` directory first:

```bash
rm -rf data
```

Then recreate the split:

```bash
python scripts/train_val_split.py \
    --datapath="custom_data" \
    --train_pct=.8
```

This is important because otherwise old images or labels can remain in `data/` and contaminate the new training or validation dataset.

A safe workflow after changing the dataset is therefore:

```bash
rm -rf data

python scripts/train_val_split.py \
    --datapath="custom_data" \
    --train_pct=.8
```

---

# 6. Generate `data.yaml`

YOLO uses `data.yaml` to locate the dataset and identify the classes.

Generate it using:

```bash
python scripts/create_data_yaml.py
```

The script reads:

```text
custom_data/classes.txt
```

and generates:

```text
data.yaml
```

A generated file will contain information similar to:

```yaml
path: /absolute/path/to/software/computer_vision/data
train: train/images
val: validation/images

nc: 2

names:
- defective
- nondefective
```

`data.yaml` contains a machine-specific absolute path.

Therefore, after cloning or moving the repository, regenerate it using:

```bash
python scripts/create_data_yaml.py
```

Do not manually copy another computer's absolute path.

---

# 7. Train YOLO26n

Train the custom detector with:

```bash
yolo detect train \
    data=data.yaml \
    model=yolo26n.pt \
    epochs=40 \
    imgsz=640 \
    device=0 \
    project=runs/detect \
    name=davinci_v1
```

The main options are:

```text
data=data.yaml     dataset configuration
model=yolo26n.pt   pretrained YOLO26 Nano model
epochs=40          number of training epochs
imgsz=640          model input image size
device=0           NVIDIA GPU 0
name=davinci_v1    experiment name
```

If no compatible GPU is available, use:

```text
device=cpu
```

Training output is stored in:

```text
runs/detect/davinci_v1/
```

The important trained weights are:

```text
runs/detect/davinci_v1/weights/
├── best.pt
└── last.pt
```

Use:

```text
best.pt
```

as the selected model because it represents the best validation performance observed during training.

---

# 8. Retraining

Do not overwrite useful previous experiments.

For another training attempt, change the run name:

```text
davinci_v1
davinci_v2
davinci_v3
...
```

For example:

```bash
yolo detect train \
    data=data.yaml \
    model=yolo26n.pt \
    epochs=40 \
    imgsz=640 \
    device=0 \
    project=runs/detect \
    name=davinci_v2
```

If the source dataset changed, remember to rebuild the dataset before retraining:

```bash
rm -rf data

python scripts/train_val_split.py \
    --datapath="custom_data" \
    --train_pct=.8

python scripts/create_data_yaml.py
```

Then begin the new training run.

---

# 9. Save the Selected Model

Create the model directory if necessary:

```bash
mkdir -p models
```

Copy the selected weights:

```bash
cp runs/detect/davinci_v1/weights/best.pt \
   models/davinci_detector.pt
```

The model used by the vision program is then:

```text
models/davinci_detector.pt
```

During development, test models do not need to be uploaded to GitHub.

Once a final model is selected for the robot/Raspberry Pi, that selected model can be versioned separately.

---

# 10. Validate the Model

Run YOLO validation:

```bash
yolo detect val \
    model=models/davinci_detector.pt \
    data=data.yaml
```

Important metrics include:

```text
precision
recall
mAP50
mAP50-95
```

Metrics should not be used alone.

Also visually inspect detections to check:

- missed defective objects
- incorrect classifications
- false detections
- bounding-box quality
- behavior under different positions and lighting

---

# 11. Test on Validation Images

Run:

```bash
python vision/yolo_detect.py \
    --model models/davinci_detector.pt \
    --source data/validation/images
```

The detector should display:

```text
class
confidence
bounding box
center (cx, cy)
```

The center is calculated from the bounding box:

```text
cx = (xmin + xmax) / 2
cy = (ymin + ymax) / 2
```

These values are currently **pixel coordinates**.

Real-world coordinate conversion will be added later during camera calibration and robot integration.

---

# 12. Run Detection + Tracking on Video

Use a video file:

```bash
python vision/yolo_detect.py \
    --model models/davinci_detector.pt \
    --source /path/to/video.mp4
```

For example:

```bash
python vision/yolo_detect.py \
    --model models/davinci_detector.pt \
    --source ~/Videos/davinci_test.mp4
```

Example output:

```text
id=3, class=defective, confidence=0.96, center=(327, 214)
id=3, class=defective, confidence=0.95, center=(344, 214)
id=3, class=defective, confidence=0.97, center=(361, 215)
```

The changing center coordinates indicate that the object is moving.

The persistent ID:

```text
id=3
```

indicates that ByteTrack believes the detections in consecutive frames belong to the same physical object.

---

# 13. Test the Camera

A simple camera test script is included:

```text
scripts/camera_test.py
```

Run:

```bash
python scripts/camera_test.py
```

Use this before YOLO when troubleshooting a USB camera.

The purpose of this script is to verify that OpenCV can receive a usable camera stream independently of YOLO inference.

---

# 14. Run with a USB Camera

For camera index `0`:

```bash
python vision/yolo_detect.py \
    --model models/davinci_detector.pt \
    --source usb0 \
    --resolution 640x480
```

For another camera index:

```text
usb1
usb2
...
```

For example:

```bash
python vision/yolo_detect.py \
    --model models/davinci_detector.pt \
    --source usb1 \
    --resolution 640x480
```

When using WSL2, the USB camera may need to be attached to WSL separately before `/dev/video0` becomes available.

Useful Linux checks include:

```bash
lsusb
```

and:

```bash
ls /dev/video*
```

To inspect supported camera formats:

```bash
v4l2-ctl \
    -d /dev/video0 \
    --list-formats-ext
```

The current detector uses OpenCV/V4L2 camera capture and MJPEG for the USB-camera path.

---

# 15. Current Vision Output

The standalone computer vision subsystem currently provides:

```text
Detection
├── track_id
├── class_name
├── confidence
├── xmin
├── ymin
├── xmax
├── ymax
├── center_x
└── center_y
```

Conceptually:

```text
Camera / Video
      ↓
YOLO26n
      ↓
Object Detection
      ↓
ByteTrack
      ↓
class
confidence
bounding box
center
tracking ID
```

This information will later be passed to ROS 2.

The vision subsystem is responsible for reporting what it sees.

Robot control, motion planning, and conveyor control remain separate subsystems.

---

# 16. Raspberry Pi 5 Deployment — Later

Raspberry Pi deployment will be completed after the desktop computer vision pipeline is finalized.

The selected PyTorch model can later be exported to **NCNN**:

```bash
yolo export \
    model=models/davinci_detector.pt \
    format=ncnn \
    imgsz=640
```

This produces an NCNN model for Raspberry Pi testing.

The target robot platform is:

```text
Raspberry Pi 5
Ubuntu 24.04
ROS 2 Jazzy
```

Future Raspberry Pi testing will evaluate:

```text
FPS
inference latency
detection accuracy
tracking stability
```

---

# Project Structure

```text
computer_vision/
├── custom_data/
│   └── classes.txt
│
├── models/
│   └── .gitkeep
│
├── scripts/
│   ├── camera_test.py
│   ├── create_data_yaml.py
│   └── train_val_split.py
│
├── vision/
│   └── yolo_detect.py
│
├── .gitignore
├── README.md
└── requirements.txt
```

The following files are generated locally and should normally not be committed:

```text
custom_data/images/
custom_data/labels/
data/
data.yaml
runs/
test dataset ZIP files
temporary videos
yolo26n.pt
test-trained model weights
```

The repository currently keeps the code necessary to:

```text
prepare a dataset
split the dataset
generate YOLO configuration
train a model
validate a model
run detection
calculate object position
track objects
test a camera
```

---

# Workflow Summary

```text
Collect images
      ↓
Label with Label Studio
      ↓
Export as YOLO
      ↓
Unzip into custom_data/
      ↓
Delete old data/ if dataset changed
      ↓
train_val_split.py
      ↓
data/
      ↓
create_data_yaml.py
      ↓
data.yaml
      ↓
Train YOLO26n
      ↓
best.pt
      ↓
davinci_detector.pt
      ↓
Validate
      ↓
Test images / video / camera
      ↓
Detection + Tracking
      ↓
class + confidence + bbox + center + tracking ID
      ↓
Raspberry Pi deployment later
      ↓
ROS 2 integration later
```

---

# References

This implementation is based on and adapted from:

- Edje Electronics / EJ Technology Consultants YOLO training workflow
- Edje Electronics `train_val_split.py`
- Edje Electronics `yolo_detect.py`
- Ultralytics YOLO
- Ultralytics ByteTrack tracking support

The original workflow was adapted for the DaVinci manufacturing project with:

```text
custom defect classes
center-coordinate output
integrated object tracking
USB camera support
future Raspberry Pi 5 deployment
future ROS 2 integration
```