#include "objectRecognizerModule.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

ObjectRecognizerModule::ObjectRecognizerModule()
{
    imagePort = NULL;
    voiceCommandTriggered = false;

    caffe_wrapper = NULL;

    // labels
    string label_file = rf.check("label_file", Value("/path/to/labels.txt")).asString().c_str();;
    cout << "Setting labels.txt to " << label_file << endl;

    ifstream infile;

    string obj_name;
    vector<string> obj_names;
    int obj_idx;
    vector<int> obj_idxs;

    infile.open (label_file.c_str());
    infile >> obj_name;
    infile >> obj_idx;
    while (!infile.eof()) {
        std::cout << obj_name << " --> "<< obj_idx << std::endl;
        obj_names.push_back(obj_name);
        obj_idxs.push_back(obj_idx);
        infile >> obj_name;
        infile >> obj_idx;
    }
    infile.close();

    if (obj_names.size()!=obj_idxs.size())
    {
        std::cout << "label_file wrongly formatted!" << std::endl;
    }

    n_classes = obj_names.size();

    labels = new string[n_classes];
    for (int i=0; i<n_classes; i++)
    {
        labels[obj_idxs[i]] = obj_names[i];
    }

    // colors
    colors.push_back(cv::Scalar( 65, 47,213));
    colors.push_back(cv::Scalar(122, 79, 58));
    colors.push_back(cv::Scalar(154,208, 72));
    colors.push_back(cv::Scalar( 71,196,249));
    colors.push_back(cv::Scalar(224,176, 96));
    colors.push_back(cv::Scalar( 22,118,238));
}

bool ObjectRecognizerModule::configure(ResourceFinder &rf)
{

    Time::turboBoost();

    // module name
    string name = rf.find("name").asString().c_str();

    // CAFFE Init
    // Binary file (.caffemodel) containing the network's weights
    string caffemodel_file = rf.check("caffemodel_file", Value("/path/to/model.caffemodel")).asString().c_str();
    cout << "Setting .caffemodel file to " << caffemodel_file << endl;

    // Text file (.prototxt) defining the network structure
    string prototxt_file = rf.check("prototxt_file", Value(" /path/to/deploy_imDataLayer.prototxt")).asString().c_str();
    cout << "Setting .prototxt file to " << prototxt_file << endl;

    // Name of blob to be extracted
    string blob_name = rf.check("blob_name", Value("prob")).asString().c_str();
    cout << "Setting blob_name to " << blob_name << endl;

    // Compute mode and eventually GPU ID to be used
    string compute_mode = rf.check("compute_mode", Value("GPU")).asString();
    int device_id = rf.check("device_id", Value(0)).asInt();

    int resize_width = rf.check("resize_width", Value(256)).asDouble();
    int resize_height = rf.check("resize_height", Value(256)).asDouble();

    caffe_wrapper = new CaffeWrapper<float>(caffemodel_file, prototxt_file,
        resize_width, resize_height,
        blob_name,
        compute_mode, device_id);

    // output and input ports
    if (!port_out_view.open(("/"+name+"/view:o").c_str()) ||
        !port_out_scores.open(("/"+name+"/scores:o").c_str()) ||
        !positionOutport->open(("/"+name+"/position:o").c_str()) ||
        !imageInport->open(("/"+name+"/camera:i").c_str()) ||
        !segmentationInport->open(("/"+name+"/crops:i").c_str())){
        return false;
    }

    // rpc ports
    if (!userPrefInport.open(("/"+name+"/userLabel/rpc:i").c_str())) {
        return false;
    }
    attach(userPrefInport);

    return true;
}

bool ObjectRecognizerModule::interruptModule()
{
    port_out_view.interrupt();
    port_out_scores.interrupt();
    positionOutport->interrupt();

    imageInport->interrupt();
    segmentationInport->interrupt();
    userPrefInport.interrupt();

    return true;
}

bool ObjectRecognizerModule::close()
{
    port_out_view.close();
    port_out_scores.close();
    positionOutport->close();

    imageInport->close();
    segmentationInport->close();
    userPrefInport.close();

    delete[] labels;

    return true;
}

bool ObjectRecognizerModule::respond(const Bottle &command, Bottle &reply)
{
    voiceCommandTriggered = true;


}

double ObjectRecognizerModule::getPeriod(){
    return 1.0;
}

bool ObjectRecognizerModule::updateModule()
{
    // First we retrieve the bounded box (and the position) of the two objects for classification
    Bottle* box_pos = segmentationInport->read();

    if (!FLAG_UNIT_TEST){ // we get the bounded boxes from segmentation module
        if (box_pos == NULL){
            mutex->lock();
            noObject = true;
            mutex->unlock();
            return false;
        }
        mutex->lock();
        noObject = false;
        mutex->unlock();
    }

    // if there are any objects to classify, we get the current frame
    Image* img = imageInport->read();
    if (img == NULL){
        return false;
    }
    cv::Mat img_mat = cv::cvarrToMat((IplImage*)img->getIplImage());
    cv::cvtColor(img_mat, img_mat, CV_RGB2BGR); // convert from RGB to BGR
    cv::Mat img_crop_mat = cv::Mat(RADIUS,RADIUS,CV_8UC3); // future cropped image

    // Crop the images around objects and classify the objects
    if (!FLAG_UNIT_TEST){ //
        int numObj = box_pos->size()/2;
        cv::Point tl(0,0);
        cv::Point br(0,0);
        bool ok = true;

        for(int i=0; i<numObj; i++){
            tl.x = box_pos->get(i*2).asList()->get(0).asInt();
            tl.y = box_pos->get(i*2).asList()->get(1).asInt();
            br.x = box_pos->get(i*2).asList()->get(2).asInt();
            br.y = box_pos->get(i*2).asList()->get(3).asInt();
            crop(img_mat, img_crop_mat, tl, br);
            float max_score = 0;
            int classObject = 0;
            ok = classify(img_crop_mat, max_score, classObject);
            if (!ok) return false;
            mutex->lock();
            classifPosMap[labels[classObject]] = box_pos->get(i*2+1).asList();
            mutex->unlock();
            classifScoreMap[labels[classObject]] = max_score;
        }
    }
    else{ // if we are in unit test mode then we take a fixed crop radius
        // extract the crop: init variables
        int x=-1;
        int y=-1;

        x = floor(img_mat.cols*0.5f);
        y = floor(img_mat.rows*0.5f);

        // extract the crop: validate the coordinates
        int r = std::min(RADIUS,x);
        r = std::min(r,y);
        r = std::min(r,img_mat.cols-x-1);
        r = std::min(r,img_mat.rows-y-1);
        if (r>10)
        {
            tl.x = x-r;
            tl.y = y-r;
            br.x = x+r;
            br.y = y+r;
        }
        // crop and classify
        crop(img_mat, img_crop_mat, tl, br);
        float max_score = 0;
        int classObject = 0;
        ok = classify(img_crop_mat, max_score, classObject);
        if (!ok) return false;
        classifScoreMap[labels[classObject]] = max_score;
    }

    // prepare outputs
    Stamp stamp;
    this->getEnvelope(stamp);

    // send out the scores
    if (port_out_scores.getOutputCount())
    {
        Bottle scores_bottle;
        for (int i=0; i<scores.size(); i++)
        {
            Bottle &b = scores_bottle.addList();
            b.addString(labels[i].c_str());
            b.addDouble(scores[i]);
        }
        port_out_scores.write(scores_bottle);
    }
    // FLAG: TRUE ok but FLAG: FALSE
    // send out the predicted label over the cropped region
    if (port_out_view.getOutputCount())
    {
        int y_text, x_text;
        y_text = tly-10;
        x_text = tlx;
        if (y_text<5)
        y_text = bry+2;

        cv::cvtColor(img_mat, img_mat, CV_RGB2BGR);
        cv::rectangle(img_mat,cv::Point(tlx,tly),cv::Point(brx,bry),cv::Scalar(0,255,0),2);
        cv::putText(img_mat,labels[max_idx].c_str(),cv::Point(x_text,y_text), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,255,0), 4);

        port_out_view.write(img);
    }

    mutex.unlock();

    return true;

}

bool ObjectRecognizer::cropImage(const cv::Mat& in_image, cv::Mat& out_image, cv::Point tl, cv::Point br){
    cv::Rect img_ROI = cv::Rect(tl, br);
    out_image.resize(img_ROI.width, img_ROI.height);
    in_image(img_ROI).copyTo(out_image);
    return true;
}

bool ObjectRecognizer::classify(const cv::Mat& image_cropped, float& max_score, int& classObject){
    std::vector<float> scores;

    // run classification with caffe
    if (!caffe_wrapper->forward(image_cropped, scores))
    {
        std::cout << "forward(): failed..." << std::endl;
        return false;
    }

    if (scores.size()!=n_classes)
    {
        std::cout << n_classes << std::endl;
        std::cout << scores.size() << std::endl;
        std::cout << "number of labels differs from number of scores!" << std::endl;
        return false;
    }

    // compute max of scores
    int max_idx = 0;
    max_score = scores[max_idx];

    for (int class_idx=1; class_idx<n_classes; class_idx++)
    {
        if (scores[class_idx] > max_score)
        {
            max_idx = class_idx;
            max_score = scores[max_idx];
        }
    }
    classObject = max_idx;
    return true;
}
