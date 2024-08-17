import cv2
import os
from img2vec_pytorch import Img2Vec
from PIL import Image
from sklearn.metrics.pairwise import cosine_similarity
import numpy as np

img2vec = Img2Vec(cuda=False, model='densenet', layer_output_size=1024)

object_str = ['Bluemoon', 'Oreo', 'Lays', 'Nongfu', 'Cola', 'Shuyuan', 'Sprite', 'Fanta', 'Libai', 'Franzzi', 'Shampoo', 'Bread']

def get_vec(idx):
    vec = []
    for file in os.listdir(f'/home/zzt/tk23_ws/src/tk23_vision/src/imgs/{object_str[idx]}/'):
        print(file)
        img = Image.open(f'/home/zzt/tk23_ws/src/tk23_vision/src/imgs/{object_str[idx]}/{file}')
        if np.asarray(img).shape[2] > 3:
            img = Image.fromarray(np.asarray(img)[:, :, :-1])
        vec.append(img2vec.get_vec(img, tensor=True))
    return np.array(vec)

for i in range(12):
    vecs = get_vec(i)
    print(vecs.shape)
    np.save(f'/home/zzt/tk23_ws/src/tk23_vision/src/object_detection/object_detection/vectors/{object_str[i]}.npy', vecs)

img0 = Image.open(f'/home/zzt/tk23_ws/src/tk23_vision/src/1.jpg')
vec1 = img2vec.get_vec(img0, tensor=True)
for idx in range(12):
    similarity_max = 0
    vec2 = np.load(f'/home/zzt/tk23_ws/src/tk23_vision/src/object_detection/object_detection/vectors/{object_str[idx]}.npy', allow_pickle=True)
    for i in range(len(vec2)):
        similarity = cosine_similarity(vec1.reshape(1, -1), vec2[i].reshape(1, -1))[0][0]
        if similarity > similarity_max:
            similarity_max = similarity
    print(object_str[idx], similarity_max)