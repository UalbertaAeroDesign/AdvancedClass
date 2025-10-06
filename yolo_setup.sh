# Create & activate a fresh env (we'll call it yolo)
conda create -n yolo python=3.10 -y
conda activate yolo

# Install PyTorch (Apple Silicon has MPS acceleration)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
# Install YOLOv8
pip install ultralytics
python - << 'PY'
import torch
print("torch:", torch.__version__)
print("MPS available:", torch.backends.mps.is_available())
PY
