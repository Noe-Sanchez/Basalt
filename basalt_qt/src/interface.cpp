#include <QApplication>
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QPainter>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QTableWidget>
#include <QHeaderView>
#include <QSpinBox>
#include <QLabel>
#include <QDebug>
#include <vector>
#include <algorithm>
#include <functional>

/* ===============================
   Circle Item
   =============================== */

class CircleItem : public QGraphicsEllipseItem
{
public:
    static bool snapEnabled;
    static int gridSize;
    static int nextID;

    int id;
    int zValueLogical = 0;
    std::function<void()> updateCallback;

    CircleItem(qreal x, qreal y, qreal r)
        : QGraphicsEllipseItem(-r, -r, 2*r, 2*r)
    {
        id = nextID++;
        setPos(x, y);

        setFlag(ItemIsMovable);
        setFlag(ItemIsSelectable);
        setFlag(ItemSendsGeometryChanges);

        updateColor();
    }

    void updateColor()
    {
        int z = zValueLogical;

        if(z == 0){
            setBrush(Qt::cyan);
            return;
        }

        int intensity = std::min(255,80 + std::abs(z)*40);

        if(z > 0)
            setBrush(QColor(intensity,50,50));
        else
            setBrush(QColor(50,50,intensity));
    }

protected:

    QVariant itemChange(GraphicsItemChange change,
                        const QVariant &value) override
    {
        if(change == ItemPositionChange && snapEnabled)
        {
            QPointF p = value.toPointF();
            int gx = qRound(p.x()/gridSize)*gridSize;
            int gy = qRound(p.y()/gridSize)*gridSize;
            return QPointF(gx,gy);
        }

        if(change == ItemPositionHasChanged)
        {
            if(updateCallback) updateCallback();
        }

        return QGraphicsEllipseItem::itemChange(change,value);
    }
};

bool CircleItem::snapEnabled = true;
int CircleItem::gridSize = 40;
int CircleItem::nextID = 0;


/* ===============================
   External Functions
   =============================== */
/*
void printCirclePositions(QGraphicsScene* scene)
{
    qDebug() << "---- Circle Positions ----";

    for(auto item : scene->items())
    {
        CircleItem* circle = dynamic_cast<CircleItem*>(item);

        if(circle)
        {
            QPointF p = circle->pos();

            double gx = p.x() / CircleItem::gridSize;
            double gy = p.y() / CircleItem::gridSize;

            qDebug() << "Circle ID:" << circle->id
                     << "X:" << gx
                     << "Y:" << gy
                     << "Z:" << circle->zValueLogical;
        }
    }
}
*/

void updateTableFromVector(const std::vector<CircleItem*>& circles,
                           QTableWidget* table)
{
    std::vector<CircleItem*> sorted = circles;

    std::sort(sorted.begin(), sorted.end(),
              [](CircleItem* a, CircleItem* b)
              {
                  return a->id < b->id;
              });

    table->setUpdatesEnabled(false);
    table->clearContents();
    table->setRowCount(sorted.size());

    for(size_t i=0;i<sorted.size();i++)
    {
        CircleItem* c = sorted[i];
        QPointF p = c->pos();

	/*
        table->setItem(i,0,new QTableWidgetItem(QString::number(c->id)));
        table->setItem(i,1,new QTableWidgetItem(QString::number(p.x())));
        table->setItem(i,2,new QTableWidgetItem(QString::number(p.y())));
        table->setItem(i,3,new QTableWidgetItem(QString::number(c->zValueLogical)));
	*/

	double gx = p.x() / CircleItem::gridSize;
        double gy = p.y() / CircleItem::gridSize;
        
        table->setItem(i,0,new QTableWidgetItem(QString::number(c->id)));
        table->setItem(i,1,new QTableWidgetItem(QString::number(gx)));
        table->setItem(i,2,new QTableWidgetItem(QString::number(gy)));
        table->setItem(i,3,new QTableWidgetItem(QString::number(c->zValueLogical)));
    }

    table->setUpdatesEnabled(true);
}


/* ===============================
   Grid View
   =============================== */

class GridView : public QGraphicsView
{
public:

    int gridSize = 40;
    bool panning = false;
    QPoint panStart;

    QSpinBox *zInput = nullptr;
    std::function<void()> updateCallback;

    GridView(QWidget *parent=nullptr)
        : QGraphicsView(parent)
    {
        setRenderHint(QPainter::Antialiasing);
        setTransformationAnchor(AnchorUnderMouse);
        setSceneRect(-2000,-2000,4000,4000);
    }

protected:

    void drawBackground(QPainter *painter,
                        const QRectF &rect) override
    {
        QPen pen(QColor(200,200,200));
        painter->setPen(pen);

        int left = int(rect.left()) - (int(rect.left()) % gridSize);
        int top  = int(rect.top())  - (int(rect.top()) % gridSize);

        for(int x=left; x<rect.right(); x+=gridSize)
            painter->drawLine(x,rect.top(),x,rect.bottom());

        for(int y=top; y<rect.bottom(); y+=gridSize)
            painter->drawLine(rect.left(),y,rect.right(),y);
    }

    void wheelEvent(QWheelEvent *event) override
    {
        QPointF scenePos = mapToScene(event->position().toPoint());
        auto item = scene()->itemAt(scenePos,QTransform());
        CircleItem* circle = dynamic_cast<CircleItem*>(item);

        if(circle && zInput)
        {
            int step = zInput->value();

            if(event->angleDelta().y()>0)
                circle->zValueLogical += step;
            else
                circle->zValueLogical -= step;

            circle->updateColor();

            if(updateCallback) updateCallback();
            return;
        }

        double scaleFactor = 1.15;

        if(event->angleDelta().y()>0)
            scale(scaleFactor,scaleFactor);
        else
            scale(1/scaleFactor,1/scaleFactor);
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        if(event->button()==Qt::MiddleButton)
        {
            panning=true;
            panStart=event->pos();
            setCursor(Qt::ClosedHandCursor);
            return;
        }

        if(event->modifiers() & Qt::ShiftModifier)
        {
            QPointF scenePos = mapToScene(event->pos());
            auto item = scene()->itemAt(scenePos,QTransform());
            CircleItem* circle = dynamic_cast<CircleItem*>(item);

            if(circle && zInput)
            {
                int step = zInput->value();

                if(event->button()==Qt::LeftButton)
                    circle->zValueLogical += step;
                else if(event->button()==Qt::RightButton)
                    circle->zValueLogical -= step;

                circle->updateColor();

                if(updateCallback) updateCallback();
                return;
            }
        }

        QGraphicsView::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        if(panning)
        {
            QPointF delta = mapToScene(panStart) -
                            mapToScene(event->pos());

            panStart = event->pos();
            setSceneRect(sceneRect().translated(delta.x(),delta.y()));
        }

        QGraphicsView::mouseMoveEvent(event);
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if(event->button()==Qt::MiddleButton)
        {
            panning=false;
            setCursor(Qt::ArrowCursor);
            return;
        }

        QGraphicsView::mouseReleaseEvent(event);
    }
};


/* ===============================
   Main Window
   =============================== */

class MainWindow : public QMainWindow
{
public:

    QGraphicsScene* scene;
    GridView* view;
    QTableWidget* table;
    QSpinBox* zInput;

    std::vector<CircleItem*> circles;

    std::function<void(QGraphicsScene*)> printHandle;
    //printHandle = printCirclePositions;

    MainWindow()
    {
        scene = new QGraphicsScene;

        view = new GridView;
        view->setScene(scene);

        table = new QTableWidget;
        table->setColumnCount(4);
        table->setHorizontalHeaderLabels({"ID","X","Y","Z"});
        table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

        QPushButton* addBtn = new QPushButton("Add Circle");
        QPushButton* removeBtn = new QPushButton("Remove Selected");
        QPushButton* snapBtn = new QPushButton("Snap Toggle");
        QPushButton* printBtn = new QPushButton("Print Positions");

        QLabel* zLabel = new QLabel("Z Distance");

        zInput = new QSpinBox;
        zInput->setRange(1,100);
        zInput->setValue(1);

        view->zInput = zInput;

        auto refresh = [this]()
        {
            updateTableFromVector(circles,table);
        };

        view->updateCallback = refresh;

        QObject::connect(addBtn,&QPushButton::clicked,[this,refresh]()
        {
            CircleItem* c = new CircleItem(0,0,20);

            c->updateCallback = refresh;

            scene->addItem(c);
            circles.push_back(c);

            refresh();
        });

        QObject::connect(removeBtn,&QPushButton::clicked,[this,refresh]()
        {
            auto sel = scene->selectedItems();

            for(auto item : sel)
            {
                CircleItem* c = dynamic_cast<CircleItem*>(item);

                if(c)
                {
                    circles.erase(
                        std::remove(circles.begin(),
                                    circles.end(),
                                    c),
                        circles.end());

                    scene->removeItem(c);
                    delete c;
                }
            }

            refresh();
        });

        QObject::connect(snapBtn,&QPushButton::clicked,[]
        {
            CircleItem::snapEnabled = !CircleItem::snapEnabled;
        });

        QObject::connect(printBtn,&QPushButton::clicked,[this]
        {
            //printCirclePositions(scene);
	    if(printHandle) printHandle(scene);
        });

        QGraphicsEllipseItem* origin =
        scene->addEllipse(-6,-6,12,12,
                          QPen(Qt::yellow),
                          QBrush(Qt::yellow));

        origin->setPos(0,0);
        origin->setZValue(100);

        QWidget* central = new QWidget;

        QHBoxLayout* mainLayout = new QHBoxLayout;
        QVBoxLayout* left = new QVBoxLayout;

        left->addWidget(addBtn);
        left->addWidget(removeBtn);
        left->addWidget(printBtn);
        left->addWidget(snapBtn);
        left->addWidget(zLabel);
        left->addWidget(zInput);
        left->addWidget(view);

        mainLayout->addLayout(left,4);
        mainLayout->addWidget(table,2);

        central->setLayout(mainLayout);

        setCentralWidget(central);

        resize(1200,700);
    }
};
