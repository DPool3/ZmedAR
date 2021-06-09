#include "videobasedview.h"
#include "ui_videobasedview.h"

VideoBasedView::VideoBasedView(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::VideoBasedView)
{
    ui->setupUi(this);
}

VideoBasedView::~VideoBasedView()
{
    delete ui;
}
