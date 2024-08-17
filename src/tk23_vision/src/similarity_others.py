import cv2
import os
from img2vec_pytorch import Img2Vec
from PIL import Image
from sklearn.metrics.pairwise import cosine_similarity
import numpy as np
from ultralytics import YOLO


model = YOLO('yolov8m-seg.pt')
img2vec = Img2Vec(cuda=False, model='densenet', layer_output_size=1024)

def vec_similarity(img1_vec, img2_vec):
    similarity = cosine_similarity(img1_vec.reshape(1, -1), img2_vec.reshape(1, -1))[0][0]
    return similarity

def get_similarity(idx, bbox, img):
    real_object = img[bbox[0]:bbox[2], bbox[1]:bbox[3]]
    similarity_max = 0.0
    img2_vec = img2vec.get_vec(Image.fromarray(cv2.cvtColor(real_object, cv2.COLOR_BGR2RGB)), tensor=True)
    # np.load(f'tk23_vision/src/object_detection/object_detection/vectors/{object_str[idx]}.npy', allow_pickle=True)
    
    for file in os.listdir(f'/home/zzt/tk23_ws/src/tk23_vision/src/imgs/{object_str[idx]}/'):
        target_img = cv2.imread(f'/home/zzt/tk23_ws/src/tk23_vision/src/imgs/{object_str[idx]}/' + file)
        print(f'/home/zzt/tk23_ws/src/tk23_vision/src/imgs/{object_str[idx]}/' + file)
        target_vec = img2vec.get_vec(Image.fromarray(cv2.cvtColor(target_img, cv2.COLOR_BGR2RGB)), tensor=True)
        similarity = vec_similarity(target_vec, img2_vec)
        if similarity > similarity_max:
            similarity_max = similarity
    return similarity_max

object_str = ['Bluemoon', 'Oreo', 'Lays', 'Nongfu', 'Cola', 'Shuyuan', 'Sprite', 'Fanta', 'Libai', 'Franzzi', 'Shampoo', 'Bread']

img = cv2.imread('/home/zzt/tk23_ws/src/tk23_vision/src/1.jpg')

results = model(img, imgsz=(img.shape[0], img.shape[1]))

new_img = np.copy(img)

for res in results:
    boxes, masks = res.boxes, res.masks
    for i in range(len(boxes.cls)):
        conf = res.boxes.conf[i].item()
        cls = model.names[int(res.boxes.cls[i])]
        mask_obj = masks[i].data.cpu().numpy().squeeze()

        # format for deepsort tracking
        mask_nonzero = np.nonzero(mask_obj)
        # bbox = x1, y1, x2, y2
        bbox = mask_nonzero[0].min(), mask_nonzero[1].min(), mask_nonzero[0].max(), mask_nonzero[1].max()

        new_img = cv2.rectangle(new_img, (bbox[1], bbox[0]), (bbox[3], bbox[2]), (0, 255, 0), 2)
        similarity_max = 0.0
        object_id = -1
        for i in range(12):
            similarity = get_similarity(i, bbox, img)
            if similarity > similarity_max:
                similarity_max = similarity
                object_id = i
        new_img = cv2.putText(new_img, f'{object_str[object_id]}: {similarity_max:.2f}', (bbox[1], bbox[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
cv2.imwrite('result_vec_similarity.jpg', new_img)
