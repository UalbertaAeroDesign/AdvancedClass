import os
import cv2
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import rcParams
from detect_white_square import detect_white_square_cv2, detect_white_square_cv2_improved
from detect_WhiteBox import detect_white_square_yolo

images_with_white_square = '../Example_Images/With_Square'
images_without_white_square = '../Example_Images/Without_Square'

#assume these take one argument; a cv2 frame and return confidence score as well as an annotated image
function_to_test = [detect_white_square_cv2, detect_white_square_cv2_improved, detect_white_square_yolo]

# test_functions will test all functions with all combinations of one function from each sublist 
# functions must take and return an image
permutations = [
#    [lambda img: img],
#    [lambda img: img, lambda img: cv2.flip(img, 0), lambda img: cv2.flip(img, 1)],
#    [lambda img: img, lambda img: add_noise(img, 10), lambda img: add_noise(img, 40)],
    [lambda img: img, lambda img: add_brightness(img, 30), lambda img: add_brightness(img, 60), lambda img: add_brightness(img, -30)]
]
# -1 for no limit
image_limit = 5

show_incorrect_images = False

confidence_threshold = 0.3

def plot_results(results_df):
    methods = []
    images = []
    for row in results_df:
        methods += [row]

    methods = methods[3:]
    results_by_image = {}

    for row in results_df.iterrows():   
        if row[1]['Image Name'] not in images:
            images += [row[1]['Image Name']]
    
    for method in methods:
        results_by_image[method] = {'TP':[0] * len(images), 'FP':[0] * len(images), 'TN':[0] * len(images), 'FN':[0] * len(images), 'correct':[0] * len(images), 'incorrect':[0] * len(images)}
    
    overall_accuracy = {'correct' : [0] * len(methods), 'incorrect' : [0] * len(methods)}

    for row in results_df.iterrows():
        for i, m in enumerate(methods):
            if float(row[1]['Ground Truth']) > confidence_threshold:
                if float(row[1][m]) > confidence_threshold:
                    #True positive
                    results_by_image[m]['TP'][images.index(row[1]['Image Name'])] += 1
                    results_by_image[m]['correct'][images.index(row[1]['Image Name'])] += 1
                    overall_accuracy['correct'][i] += 1
                else:
                    #false negative
                    results_by_image[m]['FN'][images.index(row[1]['Image Name'])] += 1
                    results_by_image[m]['incorrect'][images.index(row[1]['Image Name'])] += 1
                    overall_accuracy['incorrect'][i] += 1

            else:
                if float(row[1][m]) > confidence_threshold:
                    #False positive
                    results_by_image[m]['FP'][images.index(row[1]['Image Name'])] += 1
                    results_by_image[m]['incorrect'][images.index(row[1]['Image Name'])] += 1
                    overall_accuracy['incorrect'][i] += 1
                else:
                    #True negative
                    results_by_image[m]['TN'][images.index(row[1]['Image Name'])] += 1
                    results_by_image[m]['correct'][images.index(row[1]['Image Name'])] += 1
                    overall_accuracy['correct'][i] += 1

    barWidth = 0.25
    fig  = plt.subplots(figsize =(12, 8),tight_layout=True) 

    bars = []

    bars += [np.arange(len(images))]

    for i in range(len(methods[1:])):
        bars += [[x + barWidth for x in bars[i -1]]]

    # rgba
    correct_colors = [(0,1.0,0,1),
                      (0,0.6,0,1),
                      (0,0.2,0,1)
                      ]
    incorrect_colors = [(1.0,0,0,1),
                        (0.6,0,0,1),
                        (0.2,0,0,1)
                        ]
    
    for i, m in enumerate(methods):
        plt.bar(bars[i], results_by_image[m]['correct'], color = correct_colors[i % len(correct_colors)], width = barWidth, 
                edgecolor ='grey', label =m) 
        plt.bar(bars[i], results_by_image[m]['incorrect'], color = incorrect_colors[i % len(incorrect_colors)], width = barWidth, 
                edgecolor ='grey', label =m, bottom=results_by_image[m]['correct']) 

    plt.xlabel('Image', fontweight ='bold', fontsize = 15) 
    plt.xticks([r + barWidth for r in range(len(images))], images, rotation='vertical')

    plt.legend()
    plt.show()

    fig = plt.subplots(figsize =(12, 8),tight_layout=True)

    for i, m in enumerate(methods):
        plt.bar(methods, overall_accuracy['correct'], color = correct_colors[i % len(correct_colors)], width = barWidth, 
                edgecolor ='grey', label =m) 
        plt.bar(methods, overall_accuracy['incorrect'], color = incorrect_colors[i % len(incorrect_colors)], width = barWidth, 
                edgecolor ='grey', label =m, bottom=overall_accuracy['correct']) 
    
    #plt.legend()
    plt.show()

def add_noise(img, stddev):
    mean = 0
    noise = cv2.randn(np.zeros(img.shape[:2], np.int32), mean, stddev)
    noise = cv2.merge([noise] * 3)
    #cv2.imshow(str(stddev), noise)

    img = img.astype(np.int32)
    noisy = cv2.add(img, noise)
    noisy = cv2.normalize(noisy, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    noisy = noisy.astype(np.uint8)
    return noisy

def add_brightness(img, amount):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv[:,:,2] = cv2.add(hsv[:,:,2], amount)
    return(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))

def get_frame_from_img(image_path):
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

        images += [{'is_square' : 1, 'name': os.path.basename(image_path), 'original_frame' : get_frame_from_img(image_path), 'frames' : []}]
        image_limit -=1

    for image_path in ['/'.join((images_without_white_square, i)) for i in os.listdir(images_without_white_square)]:
        if image_limit == 0:
            break

        images += [{'is_square' : 0, 'name': os.path.basename(image_path), 'original_frame' : get_frame_from_img(image_path), 'frames' : []}]
        image_limit -=1
    

    for image in images:
        index_list = [0] * len(permutations)
        all_perms_done = False
        while not all_perms_done:
            working_frame = image['original_frame'][0].copy()
            for i, idx in enumerate(index_list):
                working_frame = permutations[i][idx](working_frame)
            
            #save to list of frames
            image['frames'] += [working_frame]

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
        
    
    labels = ['Image Name', 'Image Version', 'Ground Truth']

    for func in function_to_test:
        labels += [func.__name__]
    
    results = []
    for image in images:
        for i, frame in enumerate(image['frames']):
            frame_results = []
            for func in function_to_test:
                conf, frame_out = func(frame.copy()) # copy just in case the function is not well behaved
                frame_results += [str(round(conf, 2))]
                if ((conf < confidence_threshold and image['is_square']) or \
                    (conf > confidence_threshold and not image['is_square'])) and show_incorrect_images:
                    cv2.imshow(func.__name__ + image['name'] + '_frame=' + str(i), frame_out)

            frame_results = [image['name'], str(i), str(image['is_square'])] + frame_results
            results += [frame_results]
        print("Test " + image['name'])

    results_df = pd.DataFrame(results, columns=labels)

    print(results_df.to_markdown())
    
    plot_results(results_df)
    cv2.waitKey(0)        
    cv2.destroyAllWindows()

if __name__ == '__main__':
    test_functions(image_limit)