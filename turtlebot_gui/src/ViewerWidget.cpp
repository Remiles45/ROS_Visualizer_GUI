/*=========================================
  copyright [2023] Rebecca Miles
 ==========================================*/
#include "include/ViewerWidget.h"

Viewer_W::Viewer_W(QWidget* parent) :
    QWidget(parent) {
    /**
     * DESCRIPTION:
     *   Constructor, sets up a subscriber to the turtlebot laserscan,
     *   creates a plot in the GUI which will display the collected laserscan, updating periodically.
    */
    // create widget objects
    // buttons
    zoom_in_btn_m = new QPushButton(QIcon(zoom_in_pix_m), "", this);
    zoom_out_btn_m = new QPushButton(QIcon(zoom_out_pix_m), "", this);
    zoom_home_btn_m = new QPushButton(QIcon(zoom_home_pix_m), "", this);
    // plot
    scatter_plot_m = new QChart();
    viewer_m = new QChartView(scatter_plot_m);

    // initialize plot settings
    viewer_m->setRubberBand(QChartView::RectangleRubberBand);
    scatter_plot_m->legend()->setVisible(false);
    // create the x and y axis
    haxis = new QValueAxis(this);
    vaxis = new QValueAxis(this);

    scatter_plot_m->addAxis(haxis, Qt::AlignBottom);
    scatter_plot_m->addAxis(vaxis, Qt::AlignLeft);

    haxis->setTitleText("X (m)");
    vaxis->setTitleText("Y (m)");

    haxis->setRange(-2.5, 2.5);
    vaxis->setRange(-2.5, 2.5);

    haxis->setTickAnchor(0);
    vaxis->setTickAnchor(0);

    // create layout
    layout_m = new QGridLayout(this);

    // add viewer to layout
    layout_m->addWidget(viewer_m, 0, 0, 15, 1);
    layout_m->addWidget(zoom_in_btn_m, 1, 2, Qt::AlignTop);
    layout_m->addWidget(zoom_out_btn_m, 2, 2, Qt::AlignTop);
    layout_m->addWidget(zoom_home_btn_m, 3, 2, Qt::AlignTop);
    layout_m->columnStretch(1);

    // connect buttons
    connect(zoom_in_btn_m, &QPushButton::released, this, &Viewer_W::zoomIn);
    connect(zoom_out_btn_m, &QPushButton::released, this, &Viewer_W::zoomOut);
    connect(zoom_home_btn_m, &QPushButton::released, this, &Viewer_W::zoomHome);

    // start a timer to repaint the plot every 250 mS
    repaint_timer_m = new QTimer(this);
    connect(repaint_timer_m, &QTimer::timeout, this, &Viewer_W::repaintScan);
    repaint_timer_m->start(250);
}

Viewer_W::~Viewer_W() {
    /**
     * DESCRIPTION:
     *   Destructor, shuts down timers. 
     *   and deletes pointers that are not QWidget children
    */
    repaint_timer_m->stop();
    delete scatter_plot_m;  // not a QWidget child
}

void Viewer_W::repaintScan() {
    /**
     * DESCRIPTION: 
     *    takes the most recent laser scan and updates the plot
     *   being displayed in the GUI
    */

    auto chart_series = scatter_plot_m->series();
    if (chart_series.isEmpty()) {
        // Create a scatter series in which we will store all of the datapoints that
        // the laser scan has collected.
        scan_data_m = new QScatterSeries(this);
        scan_data_m->setMarkerShape(QScatterSeries::MarkerShapeCircle);
        scan_data_m->setMarkerSize(marker_size_m);
        addDataToSeries();
        // add the data to the plot
        scatter_plot_m->addSeries(scan_data_m);
        scan_data_m->attachAxis(haxis);
        scan_data_m->attachAxis(vaxis);
    } else {
        // update data in the plot
        scan_data_m->clear();
        addDataToSeries();
    }

    // update plot with new data
    viewer_m->update();
}

void Viewer_W::addDataToSeries() {
    /**
     * DESCRIPTION:
     *   Convert data vector into x and y points to plot
    */
    // check if the raw data vector has data (relevant for before any data has been received)
    if (!raw_data_m.empty()) {
        for (int j=0; j < raw_data_m[0].size(); j++) {
            scan_data_m->append(raw_data_m[0][j], raw_data_m[1][j]);
        }
    }
}

void Viewer_W::updateData(std::vector<std::vector<float>> &in_data) {
    /**
     * DESCRIPTION:
     *   SLOT,
     *   if this function is triggered but the in_data is empty, 
     *   either an empty range vector was received or all datapoints were
     *   invalid. In which case, throw a warning to alert the user of possible problem.
     *   Store incoming data into member variable.
     *   
     *   Additionally, a minimum of 3 datapoints are required to display the scatterplot.
    */
    if (in_data.size() < 3) {
        ROS_WARN("Received Empty or Invalid Scan");
    } else {
        raw_data_m = in_data;
    }
}

void Viewer_W::setMarkerSize(float size) {
    /**
     * DESCRIPTION:
     *   set the size of the markers displaying datapoints
    */
    marker_size_m = size;
    scan_data_m->setMarkerSize(marker_size_m);
}

void Viewer_W::zoomIn() {
    /**
     * DESCRIPTION:
     *   Triggered by button, zooms in 
    */
    scatter_plot_m->zoomIn();
}

void Viewer_W::zoomOut() {
    /**
     * DESCRIPTION:
     *   Triggered by button, zooms in 
    */
    scatter_plot_m->zoomOut();
}

void Viewer_W::zoomHome() {
    /**
     * DESCRIPTION:
     *   Triggered by button, zooms in 
    */
    scatter_plot_m->zoomReset();
}
