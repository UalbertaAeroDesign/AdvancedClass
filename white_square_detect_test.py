import os
import cv2
import numpy as np

from detect_white_square import detect_white_square_cv2
from detect_WhiteBox import detect_white_square_yolo

images_with_white_square = '../Example_Images/With_Square'
images_without_white_square = '../Example_Images/Without_Square'

#assume these take one argument; a cv2 frame and return confidence score as well as an annotated image
function_to_test = [detect_white_square_cv2, detect_white_square_yolo]

image_limit = 2

def add_noise(img, amount):
    mean = 0
    stddev = amount
    noise = np.zeros(img.shape, np.uint8)
    cv2.randn(noise, mean, stddev)

    return cv2.add(img, noise)

permutations = [
    [lambda img: img, lambda img: cv2.flip(img, 0), lambda img: cv2.flip(img, 1)],
    [lambda img: img, lambda img: add_noise(img, 180)],
]


def get_frames_from_img(image_path):
    width = 512 
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)

    #rescale to 'width' while preserving aspect ratio
    scale = width / img.shape[1]
    height = int(img.shape[0] * scale)

    resized = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)

    return [resized]

def test_functions(image_limit):
    images = []
    for image_path in ['/'.join((images_with_white_square, i)) for i in os.listdir(images_with_white_square)]:
        if image_limit == 0:
            break

        images += [{'is_square' : 1, 'name': os.path.basename(image_path), 'frames' : get_frames_from_img(image_path)}]
        image_limit -=1

    for image_path in ['/'.join((images_without_white_square, i)) for i in os.listdir(images_without_white_square)]:
        if image_limit == 0:
            break

        images += [{'is_square' : 0, 'name': os.path.basename(image_path), 'frames' : get_frames_from_img(image_path)}]
        image_limit -=1
    

    for image in images:
        index_list = [0] * len(permutations)
        all_perms_done = False
        while not all_perms_done:
            working_frame = image['frames'][0].copy()
            for i, idx in enumerate(index_list):
                working_frame = permutations[i][idx](working_frame)
            
            #save to list of frames
            image['frames'] += [working_frame]
            #cv2.imshow('', working_frame)

            #increment index list
            for i, idx in enumerate(index_list):
                idx += 1
                if len(permutations[i]) == idx:
                    if i == len(index_list) - 1:
                        all_perms_done = True
                        break
                    index_list[i] = 0
                else:
                    index_list[i] = idx
                    break
        

    print("Image name | Image Version | Ground Truth | ", end = '')
    for func in function_to_test:
        print('%s |' % func.__name__, end = '')
    print()

    for image in images:
        for i, frame in enumerate(image['frames']):
            results = []
            for func in function_to_test:
                conf, frame = func(frame)
                results += [str(round(conf, 2))]

                cv2.imshow(func.__name__ + image['name'] + '_frame=' + str(i), frame)

            print('%s | %d | %.2f | %s |' % (image['name'], i, image['is_square'], ' | '.join(results)))        
    
    cv2.waitKey(0)        
    cv2.destroyAllWindows()

if __name__ == '__main__':
    test_functions(image_limit)