from ultralytics import YOLO

# Load the YOLO11 model
model = YOLO("best_task_2.pt")

# Export the model to NCNN format
model.export(format="ncnn", imgsz=320)  # for 320
#model.export(format="ncnn") # for 640


# Load the exported NCNN model, for testing
ncnn_model = YOLO("./best_task_2_ncnn_model")