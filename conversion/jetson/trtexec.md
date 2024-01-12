
To Setup:
 1. go to /usr/src/tensorrt/samples/trtexec 
 2. sudo make 
 3. trtexec binary is in /usr/src/tensorrt/bin
 4. add to path: export PATH=$PATH:/usr/src/tensorrt/bin



trtexec --onnx=convert/landing_134.onnx --saveEngine=convert/landing_134.engine -exportOutput=convert/output.json --exportProfile=convert/profile.json --exportLayerInfo=convert/layer.json 

# only for dynamic dont use for static models
--minShapes="images":1x3x384x640 --optShapes="images":1x3x384x640 --maxShapes="images":1x3x384x640


