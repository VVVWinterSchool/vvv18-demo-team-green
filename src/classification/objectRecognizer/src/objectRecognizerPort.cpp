#include "objectRecognizerPort.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

ObjectRecognizerPort::ObjectRecognizerPort(ResourceFinder &rf) : BufferedPort<Image>()
{
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

    caffe_wrapper = NULL;
    caffe_wrapper = new CaffeWrapper<float>(caffemodel_file, prototxt_file,
        resize_width, resize_height,
        blob_name,
        compute_mode, device_id);

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

    // parameters
    radius = 256;
    img_crop_mat = cv::Mat(radius,radius,CV_8UC3);

    // this port
    BufferedPort<Image>::useCallback();

    // module name
    string name = rf.find("name").asString().c_str();

    // inout ports
    port_in_centroid.open(("/"+name+"/centroid:i").c_str());
    port_in_roi.open(("/"+name+"/roi:i").c_str());

    // output ports
    port_out_view.open(("/"+name+"/view:o").c_str());
    port_out_scores.open(("/"+name+"/scores:o").c_str());
    port_out_hist.open(("/"+name+"/hist:o").c_str());

}

void ObjectRecognizerPort::onRead(Image &img)
{
    mutex.wait();

    // If something arrived...
    if (img.width()>0 && img.height()>0)
    {
        img_mat = cv::cvarrToMat((IplImage*)img.getIplImage());
        cv::cvtColor(img_mat, img_mat, CV_RGB2BGR); // convert from RGB to BGR

        int tlx = -1;
        int tly = -1;
        int brx  = -1;
        int bry = -1;

        if(FLAG_UNIT_TEST){ // if we are in unit test mode then we take images from the webcam and crop them
            // extract the crop: init variables
            int x=-1;
            int y=-1;

            x = floor(img_mat.cols*0.5f);
            y = floor(img_mat.rows*0.5f);

            // extract the crop: validate the coordinates
            int r = std::min(radius,x);
            r = std::min(r,y);
            r = std::min(r,img_mat.cols-x-1);
            r = std::min(r,img_mat.rows-y-1);
            if (r>10)
            {
                tlx = x-r;
                tly = y-r;
                brx = x+r;
                bry = y+r;
            }

            // crop the image
            cv::Rect img_ROI = cv::Rect(cv::Point( tlx, tly ), cv::Point( brx, bry ));
            img_crop_mat.resize(img_ROI.width, img_ROI.height);
            img_mat(img_ROI).copyTo(img_crop_mat);
        }
        else{ // streaming cropped image, no need to crop
            img_crop_mat.resize(img_mat.cols, img_mat.rows);
            img_mat.copyTo(img_crop_mat);
        }

        // extract the scores
        std::vector<float> scores;
        if (!caffe_wrapper->forward(img_crop_mat, scores))
        {
            std::cout << "forward(): failed..." << std::endl;
            mutex.post();
            return;
        }
        if (scores.size()!=n_classes)
        {
            std::cout << n_classes << std::endl;
            std::cout << scores.size() << std::endl;
            std::cout << "number of labels differs from number of scores!" << std::endl;
            mutex.post();
            return;
        }

        // compute max of scores
        int max_idx = 0;
        float max_score = scores[max_idx];

        for (int class_idx=1; class_idx<n_classes; class_idx++)
        {
            if (scores[class_idx] > max_score)
            {
                max_idx = class_idx;
                max_score = scores[max_idx];
            }
        }

        // print the scores
        std::cout << "SCORES: " << endl;
        for (int i=0; i<n_classes; i++)
        std::cout << "[" << labels[i] << "]: " << scores[i] << std::endl;
        std::cout << std::endl << std::endl;

        // prepare outputs
        Stamp stamp;
        this->getEnvelope(stamp);

        // send out the histogram
        if (port_out_hist.getOutputCount()>0)
        {
            // init dims
            int img_hist_height = 600;
            int img_hist_width = 800;
            ImageOf<PixelRgb> img_hist;
            img_hist.resize(img_hist_width,img_hist_height);
            img_hist.zero();

            int bin_width = img_hist.width()/n_classes;
            int bin_bottom = img_hist_height;

            // int auxiliary images
            cv::Mat img_hist_mat = cv::cvarrToMat(img_hist.getIplImage());
            cv::Mat img_text_mat = cv::Mat::zeros(img_hist.width(), img_hist.height(), CV_8UC3);

            // draw
            for (int bin_idx=0; bin_idx<n_classes; bin_idx++)
            {
                int bin_top = (int)(img_hist_height*(1.0f - scores[bin_idx]));
                cv::rectangle(img_hist_mat, cv::Point(bin_idx*bin_width,bin_top),
                cv::Point((bin_idx+1)*bin_width,bin_bottom),
                colors[bin_idx%(int)colors.size()],CV_FILLED);
            }
            for (int bin_idx=0; bin_idx<n_classes; bin_idx++)
            {
                cv::putText(img_text_mat,labels[bin_idx].c_str(),
                cv::Point(img_hist_height - bin_bottom, bin_idx*bin_width+bin_width/2),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,255,255), 2);
            }
            transpose(img_text_mat, img_text_mat);
            flip(img_text_mat, img_text_mat, 0);
            img_hist_mat = img_hist_mat + img_text_mat;

            port_out_hist.write(img_hist);

        }

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
    }

    mutex.post();
}

bool ObjectRecognizerPort::set_radius(int _radius)
{
    if (_radius>0)
    {
        mutex.wait();
        radius = _radius;
        mutex.post();
        return true;
    }
    else
    return false;
}

bool ObjectRecognizerPort::get_radius(int &_radius)
{
    mutex.wait();
    _radius = radius;
    mutex.post();
}

void ObjectRecognizerPort::interrupt()
{
    mutex.wait();

    BufferedPort<Image>::interrupt();

    port_in_centroid.interrupt();
    port_in_roi.interrupt();

    port_out_view.interrupt();
    port_out_scores.interrupt();
    port_out_hist.interrupt();

    mutex.post();
}

void ObjectRecognizerPort::resume()
{
    mutex.wait();

    BufferedPort<Image>::resume();

    port_in_centroid.resume();
    port_in_roi.resume();

    port_out_view.resume();
    port_out_scores.resume();
    port_out_hist.resume();

    mutex.post();
}

void ObjectRecognizerPort::close()
{
    mutex.wait();

    BufferedPort<Image>::close();

    port_in_centroid.close();
    port_in_roi.close();

    port_out_view.close();
    port_out_scores.close();
    port_out_hist.close();

    delete[] labels;

    mutex.post();
}
