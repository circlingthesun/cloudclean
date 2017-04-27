#include "cloudgldata.h"
#include <boost/make_shared.hpp>
#include <QDebug>

CloudGLData::CloudGLData(boost::shared_ptr<PointCloud> pc) {
    // Assumption: cloud size does not change
    pc_ = pc;

    dirty_labels_ = true;
    dirty_points_ = true;
    dirty_flags_ = true;
    dirty_grid_ = true;


    //
    // Point buffer setup
    //
    point_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    point_buffer_->setUsagePattern(QGLBuffer::StreamDraw);
    point_buffer_->create(); CE();
    point_buffer_->bind(); CE();
    size_t vb_size = sizeof(pcl::PointXYZRGB)*pc->size();
    point_buffer_->allocate(vb_size); CE();
    point_buffer_->release(); CE();
    //
    // Label buffer setup
    //
    label_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    label_buffer_->create(); CE();
    label_buffer_->bind(); CE();
    label_buffer_->allocate(pc->size()*sizeof(int16_t)); CE();
    label_buffer_->release(); CE();
    //
    // Set up flag buffer
    //
    flag_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    flag_buffer_->create(); CE();
    flag_buffer_->bind(); CE();
    size_t sb_size = sizeof(uint8_t)*pc->size();
    flag_buffer_->allocate(sb_size); CE();
    flag_buffer_->release(); CE();

    //
    // Set up grid position buffer
    //
    grid_buffer_.reset(new QGLBuffer(QGLBuffer::VertexBuffer)); CE();
    grid_buffer_->create(); CE();
    grid_buffer_->bind(); CE();
    size_t gb_size = sizeof(float)*2*pc->size();
    grid_buffer_->allocate(gb_size); CE();
    grid_buffer_->release(); CE();

    QMetaObject::invokeMethod(this, "syncCloud");
    QMetaObject::invokeMethod(this, "syncFlags");
    QMetaObject::invokeMethod(this, "syncLabels");


    connect(pc_.get(), SIGNAL(flagUpdate(boost::shared_ptr<std::vector<int> >)),
            this, SLOT(syncFlags(boost::shared_ptr<std::vector<int> >)));
    connect(pc_.get(), SIGNAL(labelUpdate(boost::shared_ptr<std::vector<int> >)),
            this, SLOT(syncLabels(boost::shared_ptr<std::vector<int> >)));
}

CloudGLData::~CloudGLData() {
    qDebug() << "CloudGLData deleted";
    disconnect(pc_.get(), SIGNAL(flagUpdate(boost::shared_ptr<std::vector<int> >)),
               this, SLOT(syncFlags(boost::shared_ptr<std::vector<int> >)));
    disconnect(pc_.get(), SIGNAL(labelUpdate(boost::shared_ptr<std::vector<int> >)),
               this, SLOT(syncLabels(boost::shared_ptr<std::vector<int> >)));
}

/*
void CloudGLData::setVAO(GLuint vao){
    glBindVertexArray(vao);

    // Point buffer
    point_buffer_->bind(); CE();
    glEnableVertexAttribArray(0); CE();
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(float)*4, 0); CE();
    glEnableVertexAttribArray(1); CE();
    int offset = sizeof(float)*3;
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(float)*4,
                          reinterpret_cast<const void *>(offset)); CE();
    point_buffer_->release(); CE();

    // Label buffer
    label_buffer_->bind(); CE();
    glEnableVertexAttribArray(2); CE(); CE();
    glVertexAttribIPointer(2, 1, GL_SHORT, 0, 0); CE();
    label_buffer_->release(); CE();

    // Flag buffer
    flag_buffer_->bind(); CE();
    glEnableVertexAttribArray(3); CE();
    glVertexAttribIPointer(3, 1, GL_BYTE, 0, 0); CE();
    flag_buffer_->release(); CE();

    // Grid pos buffer
    grid_buffer_->bind(); CE();
    glEnableVertexAttribArray(4); CE();
    glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, 0, 0); CE();
    grid_buffer_->release(); CE();

    glBindVertexArray(0);
}
*/

void CloudGLData::copyCloud(){
    point_buffer_->bind(); CE();
    size_t vb_size = sizeof(pcl::PointXYZRGB)*pc_->size();
    point_buffer_->allocate(vb_size); CE();
    float * pointbuff =
            static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();

    for(uint i = 0; i < pc_->size(); i++) {
        pointbuff[i*4] = (*pc_)[i].x;
        pointbuff[i*4+1] = (*pc_)[i].y;
        pointbuff[i*4+2] = (*pc_)[i].z;
        pointbuff[i*4+3] = (*pc_)[i].data[3];
    }

    glUnmapBuffer(GL_ARRAY_BUFFER);

//    for(uint i = 0; i < pc_->size(); i++) {
//        point_buffer_->write(i*sizeof(float)*4, (*pc_)[i].data, sizeof(float)*3);
//        point_buffer_->write(i*sizeof(float)*4, &((*pc_)[i].a), sizeof(float));
//    }

    point_buffer_->release(); CE();
}

void CloudGLData::copyLabels(){
    label_buffer_->bind(); CE();
    label_buffer_->allocate(pc_->size()*sizeof(int16_t)); CE();
    int16_t * layerbuff =
            static_cast<int16_t *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();

    if(dirty_label_list_.get() == nullptr){
        for(uint i = 0; i < pc_->labels_.size(); i++){
            layerbuff[i] = pc_->labels_[i];
        }
    } else {
        for(int i : *dirty_label_list_){
            layerbuff[i] = pc_->labels_[i];
        }
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    label_buffer_->release(); CE();
}

void CloudGLData::copyFlags(){
    flag_buffer_->bind(); CE();
    uint8_t * flag_buffer =
            static_cast<uint8_t *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();

    if(dirty_flag_list_.get() == nullptr){
        for(uint i = 0; i < pc_->size(); i++)
            flag_buffer[i] = static_cast<uint8_t>(pc_->flags_[i]);
    } else {
        for(int i : *dirty_flag_list_)
            flag_buffer[i] = static_cast<uint8_t>(pc_->flags_[i]);
    }

    glUnmapBuffer(GL_ARRAY_BUFFER);
    flag_buffer_->release(); CE();
}

void CloudGLData::copyGrid(){
    grid_buffer_->bind(); CE();
    float * gridbuff =
            static_cast<float *>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY)); CE();
    for(uint i = 0; i < pc_->cloudToGridMap().size(); i++){
        int grid_idx = pc_->cloudToGridMap()[i];
        float y = grid_idx % pc_->scan_height();
        float x = grid_idx/float(pc_->scan_height());

        //if(i%1000 == 0)
        //    qDebug() << "x, y = " << x << y;
        gridbuff[i*2] = x;
        gridbuff[i*2+1] = y;
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
    grid_buffer_->release(); CE();
}

void CloudGLData::syncCloud() {
    dirty_points_ = true;
    emit updated();
}

void CloudGLData::syncLabels(boost::shared_ptr<std::vector<int> > idxs) {
    if(dirty_labels_ && dirty_label_list_.get() != nullptr && dirty_label_list_->size() !=0){
        dirty_label_list_->insert(dirty_label_list_->end(), idxs->begin(), idxs->end());
    }
    else {
        dirty_label_list_ = idxs;
    }
    dirty_labels_ = true;
    emit updated();
}

void CloudGLData::syncFlags(boost::shared_ptr<std::vector<int> > idxs) {

    if(idxs == nullptr) {
        qDebug() << "Slow syncining all flags!";

        if(dirty_flag_list_.get() == nullptr)
            dirty_flag_list_ = boost::make_shared<std::vector<int> >();

        for(int i = 0; i < pc_->flags_.size(); i++)
            dirty_flag_list_->push_back(i);
    }
    else if(dirty_flags_ && dirty_flag_list_.get() != nullptr && dirty_flag_list_->size() !=0){
        dirty_flag_list_->insert(dirty_flag_list_->end(), idxs->begin(), idxs->end());
    }
    else if(idxs != nullptr) {
        dirty_flag_list_ = idxs;
    }
    else {
        qDebug() << "yeah... how did you get here?";
        return;
    }
    dirty_flags_ = true;
    emit updated();
}

void CloudGLData::draw(GLint vao){
    // Assumptions:
    // - shader is loaded
    // - buffertexure is loaded

    if(dirty_points_){
        copyCloud();
        dirty_points_ = false;
    }
    if(dirty_labels_){
        copyLabels();
        dirty_labels_ = false;
    }
    if(dirty_flags_){
        copyFlags();
        dirty_flags_ = false;
        dirty_flag_list_.reset();
    }
    if(dirty_grid_){
        copyGrid();
        dirty_grid_ = false;
    }

    glBindVertexArray(vao); CE();
    glDrawArrays(GL_POINTS, 0, pc_->size()); CE();
    glBindVertexArray(0); CE();
}
