from ultralytics import YOLO

# Load a model
model = YOLO("detection_models\yolo26n.pt")  # load a pretrained model (recommended for training)

# Train the model
#results = model.train(data="yolo_images\Trimmed_Images\labelme_json\YOLODataset\dataset.yaml", epochs=100, imgsz=640)
results = model.train(data="coco.yaml", epochs=100, imgsz=640)