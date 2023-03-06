#include "ViewerWidget.h"

Viewer_W::Viewer_W(QWidget* parent) : 
    QWidget(parent)
    {
    /*
    Description:
        Constructor, sets up a subscriber to the turtlebot laserscan,
        creates a plot in the GUI which will display the collected laserscan, updating periodically.
    */
    // create a plot and add it to the widget
    scatter_plot_m = new QChart();
    viewer = new QChartView(scatter_plot_m);
    QHBoxLayout *hLayout = new QHBoxLayout(this);
    hLayout->addWidget(viewer,4);
    
    // start a timer to repaint the plot every 250 mS
    repaintTimer = new QTimer(this);
    connect(repaintTimer, &QTimer::timeout, this, &Viewer_W::repaintScan);
    repaintTimer->start(250); 
}
Viewer_W::~Viewer_W(){
    /*
    Description:
        Destructor, shuts down timers.
    */
    repaintTimer->stop();
}


void Viewer_W::repaintScan(){
    /*
    Description: 
        takes the most recent laser scan and updates the plot 
        being displayed in the GUI
    */
    
    QList chart_series = scatter_plot_m->series();
    if (chart_series.isEmpty()){
        // Create a scatter series in which we will store all of the datapoints that 
        // the laser scan has collected. 
        scan_data_m = new QScatterSeries(); 
        scan_data_m->setMarkerShape(QScatterSeries::MarkerShapeCircle);
        scan_data_m->setMarkerSize(marker_size_m);
        addDataToSeries();
        // add the data to the plot
        scatter_plot_m->addSeries(scan_data_m);
        scatter_plot_m->createDefaultAxes();
    }else{
        // update data in the plot
        scan_data_m->clear();
        addDataToSeries();
    }
    
    // update plot with new data
    viewer->update();
}

void Viewer_W::addDataToSeries(){
    /*
    Description:
        Convert data vector into x and y points to plot
    */ 
    if(!raw_data_m.empty()){
        for(int j=0;j<raw_data_m[0].size();j++){
            scan_data_m->append(raw_data_m[0][j],raw_data_m[1][j]);
        }
    }
}
void Viewer_W::updateData(std::vector<std::vector<float>> &in_data){
    /*
    Description:
        Store incoming data into member variable.
    */ 
    raw_data_m = in_data;
}

void Viewer_W::setMarkerSize(float size){
    /*
    Description: 
        set the size of the markers displaying datapoints
    */
    marker_size_m = size;
    scan_data_m->setMarkerSize(marker_size_m);
}

