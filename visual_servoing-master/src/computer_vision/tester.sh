#! /bin/bash


echo "Good job! Usage: choose [n] for one image or choose '*' for all images"

function choose () {
    mkdir -p test_images_cone/archive
    mv test_images_cone/archive/* test_images_cone
    mv test_images_cone/test*.jpg test_images_cone/archive
    mv test_images_cone/archive/test$1.jpg test_images_cone
    python cv_test.py cone color
}
