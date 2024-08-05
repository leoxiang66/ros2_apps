g++ -I/usr/local/TensorRT/include -I/usr/local/cuda/include \
    -L/usr/local/TensorRT/lib -L/usr/local/cuda/lib64 \
    -lnvinfer -lcudart test_tensorrt.cpp -o test_tensorrt
nvcc -I/usr/local/TensorRT/include -I/usr/local/cuda/include -I/usr/include/opencv4 \
     -L/usr/local/TensorRT/lib -L/usr/local/cuda/lib64 \
     -lnvinfer -lcudart -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_dnn -lopencv_imgcodecs \
     infer.cpp -o infer