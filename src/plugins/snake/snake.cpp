#include "plugins/snake/snake.h"
#include <QDebug>
#include <QEvent>
#include <QKeyEvent>
#include <QAction>
#include <QGLShaderProgram>
#include <QGLBuffer>
#include <QTabWidget>
#include <QToolBar>
#include <QVBoxLayout>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QSpacerItem>
#include <QStackedWidget>
#include <QSlider>
#include <QDockWidget>
#include <QApplication>
#include <QScrollArea>
#include <boost/make_shared.hpp>
#include "model/layerlist.h"
#include "model/cloudlist.h"
#include "gui/glwidget.h"
#include "gui/flatview.h"
#include "gui/mainwindow.h"
#include "utilities/pointpicker.h"
#include "commands/select.h"
#include "pluginsystem/core.h"
#include "utilities/cv.h"
#include "plugins/snake/algo.h"

QString Snake::getName(){
    return "Snake Tool";
}

void Snake::initialize(Core *core){
    core_= core;
    cl_ = core_->cl_;
    ll_ = core_->ll_;
    glwidget_ = core_->mw_->glwidget_;
    flatview_ = core_->mw_->flatview_;
    mw_ = core_->mw_;

    enable_ = new QAction(QIcon(":/images/snake.png"), "2d snake lasso tool", 0);
    enable_->setCheckable(true);
    enable_->setChecked(false);

    is_enabled_ = false;
    min_segment_len_ = 50;

    connect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    connect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));

    mw_->addMenu(enable_, "Edit");
    mw_->toolbar_->addAction(enable_);

    settings_ = new QWidget();
    QVBoxLayout * layout = new QVBoxLayout(settings_);
    settings_->setLayout(layout);

    mw_->tooloptions_->addWidget(settings_);

    lasso_ = new Lasso();

    image_ = nullptr;
    widget_ = new QWidget(0);
    tab_idx_ = core_->mw_->addTab(widget_, "Snake viz");

    QVBoxLayout * layout2 = new QVBoxLayout(widget_);
    QScrollArea * scrollarea = new QScrollArea();
    scrollarea->setBackgroundRole(QPalette::Dark);
    layout2->addWidget(scrollarea);
    image_container_ = new QLabel();
    scrollarea->setWidget(image_container_);
}

void Snake::cleanup(){
    mw_->removeTab(tab_idx_);
    delete widget_;
    mw_->removeMenu(enable_, "Edit");
    mw_->toolbar_->removeAction(enable_);
    disconnect(this, SIGNAL(enabling()), core_, SIGNAL(endEdit()));
    disconnect(enable_, SIGNAL(triggered()), this, SLOT(enable()));
    delete lasso_;
}

void Snake::paint(){
    lasso_->drawLasso(last_mouse_pos_.x(), last_mouse_pos_.y(), flatview_);
}

void Snake::drawFloats(boost::shared_ptr<const std::vector<float> > out_img, boost::shared_ptr<PointCloud> cloud){
    // translates grid idx to cloud idx
    boost::shared_ptr<const std::vector<int>> lookup = cloud->gridToCloudMap();

    if(image_ != nullptr)
        delete image_;
    image_ = new QImage(cloud->scan_width(), cloud->scan_height(), QImage::Format_Indexed8);

    for(int i = 0; i < 256; i++) {
        image_->setColor(i, qRgb(i, i, i));
    }

    float min, max;
    min_max(*out_img, min, max);
    qDebug() << "Minmax" << min << max;

    // Draw image
    auto select = boost::make_shared<std::vector<int> >();
    for(int y = 0; y < cloud->scan_height(); y++){
        for(int x = 0; x < cloud->scan_width(); x++){
            int i = (cloud->scan_height() -1 - y) + x * cloud->scan_height();

            // Mask disabled
            if(lookup->at(i) == -2) {
                image_->setPixel(x, y, 0);
                continue;
            }

            //int intensity = 255 * (1 - distmap[i]/max_dist);
            float mag = (*out_img)[i];
            //int intensity = 255 * (1 - (mag - min)/(max - min));

            int intensity = 255 * (mag - min)/(max - min);

            if(intensity > 255 || intensity < 0) {
                qDebug() << "Nope, sorry > 255 || < 0: " << mag;
                qDebug() << "Mag: " << mag;
                qDebug() << "Intensity" << intensity;
                return;
            }

            // Select
            if(lookup->at(i) != -1 && intensity > 100) {
                select->push_back(lookup->at(i));
            }

            image_->setPixel(x, y, intensity);
        }
    }
    image_container_->setPixmap(QPixmap::fromImage(*image_));
    image_container_->resize(image_->size());

}

bool Snake::mouseClickEvent(QMouseEvent * event){
    lasso_->addScreenPoint(event->x(), event->y(), core_->mw_->flatview_->width(), core_->mw_->flatview_->height());
    return true;
}

bool Snake::mouseDblClickEvent(QMouseEvent * event){
    if(cl_->clouds_.size() == 0){
        disable();
        return false;
    }

    auto cloud = cl_->active_;

    boost::shared_ptr<std::vector<int> > empty = boost::make_shared<std::vector<int>>();
    boost::shared_ptr<std::vector<int>> selected_indices = boost::make_shared<std::vector<int>>();
    boost::shared_ptr<std::vector<int>> removed_indices= boost::make_shared<std::vector<int>>();


    // This is where the contour is modified

    // Distance between contour points should initially be less than D distance
    // Points need to be added on lines where this is not true
    // Assert points > 11 so the 4th order derivative works
    // Create an image to run the algorithm on
    // Down sampling might be required?
    // Use fuctions from utils.cpp
    // For each point calculate the internal and external energy
    // https://en.wikipedia.org/wiki/Active_contour_model
    // Calcuate x and y derivative seperately
    // Find appropriate weights
    //


    std::vector<Eigen::Vector2f> points = lasso_->getPoints();
    std::vector<Eigen::Vector2i> img_points;

    Lasso * new_lasso = new Lasso();

    Eigen::Affine2f t = flatview_->getNDCCamera();

    // Convert lasso to image space
    for(uint idx1 = 0; idx1 < points.size(); idx1++) {
        int idx2 = (idx1 + 1) % points.size();

        qDebug() << "Point added at (ndc space): " << points[idx1].x() << points[idx1].y();

        // Back to ndc scale with rotation and translation applied
        Eigen::Vector2f p1f = t.inverse() * points[idx1];
        Eigen::Vector2f p2f = t.inverse() * points[idx2];

        qDebug() << "Point added at (ndc space): " << p1f.x() << p1f.y();

        // Scale to image size
        Eigen::Vector2i p1 = Lasso::getScreenPoint(p1f,
                                   cl_->active_->scan_width(),
                                   cl_->active_->scan_height());
        Eigen::Vector2i p2 = Lasso::getScreenPoint(p2f,
                                   cl_->active_->scan_width(),
                                   cl_->active_->scan_height());


        img_points.push_back(p1);
        qDebug("Point added at (image space, %d %d): %d, %d", cl_->active_->scan_width(), cl_->active_->scan_height(),
               p1.x(), p1.y());

        float dist = (p2-p1).cast<float>().norm();

        if(dist <= min_segment_len_)
        continue;

        Eigen::Vector2f dir = (p2-p1).cast<float>().normalized();

        float dist_along_segment = 0;

        while (dist_along_segment+min_segment_len_ < dist) {
            dist_along_segment += min_segment_len_;
            Eigen::Vector2f new_point = dist_along_segment * dir + p1.cast<float>();
            img_points.push_back(new_point.cast<int>());
        }
    }


    int w = cloud->scan_width();
    int h = cloud->scan_height();

    // Create distance map
    img_ = makeDistmap(cloud);
    img_ = gradientImage(img_, h, w);
    //img_ = convolve(img_, h, w, gaussian, 5);
    //for(int i = 0; i < 3; i++)
    //    img_ = convolve(img_, h, w, gaussian, 5);

    //for(Eigen::Vector2i & p : img_points) {
    //    qDebug() << "Out: " << p.x() << p.y();
    //}


    int it = 0;
    bool converged = false;

    while(!converged && it++ < 100) {

        // Note, inverted width and height
        converged = snake_iteration(img_,
                      w,
                      h,
                      img_points,
                      21,
                      21,
                      1,
                      0.5,
                      1,
                      0.0);


        new_lasso = new Lasso();

        for(Eigen::Vector2i p : img_points) {

            Eigen::Vector2f ndc_point = t * Lasso::NDCPoint(p, cl_->active_->scan_width(), cl_->active_->scan_height());
            new_lasso->addNormPoint(ndc_point);
        }

        delete lasso_;
        lasso_ = new_lasso;

        qDebug() << it;

        flatview_->update(0, 0 , flatview_->width(), flatview_->height());
        QApplication::processEvents();
    }

/*
    for(Eigen::Vector2i p : img_points) {
        int x = p.x();
        int y = p.y();
        //(*img_)[cloud->scan_height() * x + y] = 255;
    }
*/

    drawFloats(img_, cloud);


   // points are now control points

   // for each point around the control groups, calculate the cost function,
   // move to the lowest?


/*
    lasso_->getIndices2D(cloud->scan_height(), flatview_->getCamera(),
                         cloud->cloudToGridMap(), selected_indices,
                         removed_indices);

    core_->us_->beginMacro("Snake lasso tool");
    core_->us_->push(new Select(cl_->active_, selected_indices, empty));
    core_->us_->endMacro();

    disable();
*/
    return true;
}

bool Snake::mouseMoveEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();

    if(cl_->clouds_.size() == 0) {
        disable();
        return false;
    }

    lasso_->moveLastScreenPoint(event->x(), event->y(), core_->mw_->flatview_);
    flatview_->update();

    if(event->buttons() != Qt::LeftButton)
        return false;

    flatview_->update();
    return true;
}

bool Snake::mousePressEvent(QMouseEvent * event) {
    last_mouse_pos_ << event->x(), event->y();
    mouse_down_pos_ = last_mouse_pos_;
    if(event->buttons() != Qt::LeftButton)
        return false;
    if(cl_->clouds_.size() == 0){
        disable();
        return false;
    }

    return true;
}

bool Snake::mouseReleaseEvent(QMouseEvent * event){
    last_mouse_pos_ << event->x(), event->y();
    float dist = (last_mouse_pos_ - mouse_down_pos_).norm();
    if(dist < 5){
        return mouseClickEvent(event);
    }

    return true;
}

void Snake::enable() {
    if(is_enabled_){
        disable();
        return;
    }
    QTabWidget * tabs = qobject_cast<QTabWidget *>(flatview_->parent()->parent());
    tabs->setCurrentWidget(flatview_);
    enable_->setChecked(true);

    mw_->options_dock_->show();
    mw_->tooloptions_->setCurrentWidget(settings_);

    lasso_->clear();

    emit enabling();
    connect(flatview_, SIGNAL(pluginPaint()),
            this, SLOT(paint()),
            Qt::DirectConnection);
    flatview_->installEventFilter(this);
    connect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    is_enabled_ = true;
}

void Snake::disable() {
    enable_->setChecked(false);
    disconnect(core_, SIGNAL(endEdit()), this, SLOT(disable()));
    disconnect(flatview_, SIGNAL(pluginPaint()),
            this, SLOT(paint()));
    flatview_->removeEventFilter(this);
    is_enabled_ = false;
}

bool Snake::eventFilter(QObject *object, QEvent *event){

    // Bypass plugin via shift
    if(QApplication::keyboardModifiers() == Qt::SHIFT)
        return false;

    switch(event->type()){
    case QEvent::MouseButtonPress:
        return mousePressEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonRelease:
        return mouseReleaseEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseMove:
        return mouseMoveEvent(static_cast<QMouseEvent*>(event));
    case QEvent::MouseButtonDblClick:
        return mouseDblClickEvent(static_cast<QMouseEvent*>(event));
    default:
        return false;
    }
}

Q_PLUGIN_METADATA(IID "za.co.circlingthesun.cloudclean.iplugin")
