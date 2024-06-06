#ifndef QMULTICOMBOBOX_H
#define QMULTICOMBOBOX_H

#include <QListWidget>
#include <QCheckBox>
#include <QList>
#include <QLineEdit>
#include <QComboBox>

class QListWidgetItem;
class QMultiComboBoxPrivate;
class QMultiComboBox : public QComboBox
{
    Q_OBJECT
public:
    explicit QMultiComboBox(QWidget *parent = NULL);

    /**
     * @brief addDataItem 添加数据
     * @param text
     * @param userData
     */
    void addDataItem(const QString &text, const QVariant &userData = QVariant());

    /**
     * @brief setSelectedData 设置选中的数据
     * @param selectedData
     */
    void setSelectedData(const QStringList &selectedData);

    /**
     * @brief setPopupViewHeight 设置下拉列表弹窗的高度，默认值100
     * @param height
     */
    void setPopupViewHeight(int height);

    /**
     * @brief selectedDataText 获取选择的数据
     * @return 选择的数据
     */
    QStringList selectedDataText();

    /**
     * @brief selectedDataIndex 获取选择数据的索引
     * @return 选择数据的索引
     */
    QList<int> selectedDataIndex();

    /**
     * @brief selectedUserData 获取选择数据对应的用户数据
     * @return 选择数据对应的用户数据
     */
    QList<QVariant> selectedUserData();

private slots:
    void slot_itemChanged(QListWidgetItem *item);

private:
    QMultiComboBoxPrivate *m_pd;
};

class QMultiComboBoxPrivate
{
public:
    QStringList allDataList;
    QStringList selectedDataList;
    QList<int> selectedDataIndex;
    QListWidget *pListWidget;
};

QMultiComboBox::QMultiComboBox(QWidget *parent)
    : QComboBox(parent), m_pd(new QMultiComboBoxPrivate)
{
    m_pd->pListWidget = new QListWidget;
    m_pd->pListWidget->setFixedHeight(100);
    connect(m_pd->pListWidget, SIGNAL(itemChanged(QListWidgetItem *)), this, SLOT(slot_itemChanged(QListWidgetItem *)));
    setView(m_pd->pListWidget);
    // 设置Editable为true，lineEdit() 函数才不返回空
    setEditable(true);
    lineEdit()->setReadOnly(true);
}

void QMultiComboBox::addDataItem(const QString &text, const QVariant &userData)
{
    m_pd->pListWidget->blockSignals(true);
    QListWidgetItem *pItem = new QListWidgetItem(text, m_pd->pListWidget);
    pItem->setData(Qt::UserRole, userData);
    pItem->setCheckState(Qt::Unchecked);
    m_pd->allDataList.append(text);
    // 设置QListWidgetItem 可交互且可以选中和取消选中
    pItem->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled);
    QComboBox::addItem(text);
    QComboBox::setCurrentText("");
    m_pd->pListWidget->blockSignals(false);
}

void QMultiComboBox::setSelectedData(const QStringList &selectedData)
{
    m_pd->selectedDataList.clear();
    m_pd->selectedDataList += selectedData;

    QString text = "";
    for (int var = 0; var < m_pd->selectedDataList.size(); ++var)
    {
        m_pd->pListWidget->blockSignals(true);
        QListWidgetItem *pItem = m_pd->pListWidget->item(var);
        pItem->setCheckState(Qt::Checked);
        m_pd->pListWidget->blockSignals(false);

        text.append(m_pd->selectedDataList[var]);
        if (var < m_pd->selectedDataList.size() - 1)
        {
            text.append(",");
        }
    }

    lineEdit()->setText(text);
}

QStringList QMultiComboBox::selectedDataText()
{
    return m_pd->selectedDataList;
}

QList<int> QMultiComboBox::selectedDataIndex()
{
    return m_pd->selectedDataIndex;
}

QList<QVariant> QMultiComboBox::selectedUserData()
{
    QList<QVariant> dataList;
    for (int var = 0; var < m_pd->selectedDataList.size(); ++var)
    {
        QListWidgetItem *pItem = m_pd->pListWidget->item(var);
        if (pItem->checkState() == Qt::Checked)
        {
            dataList.append(pItem->data(Qt::UserRole));
        }
    }

    return dataList;
}

void QMultiComboBox::slot_itemChanged(QListWidgetItem *item)
{
    
    if (m_pd->selectedDataList.contains(item->text()))
    {
        m_pd->selectedDataIndex.removeOne(m_pd->allDataList.indexOf(item->text()));
        m_pd->selectedDataList.removeOne(item->text());
    }
    else
    {
        m_pd->selectedDataIndex.append(m_pd->allDataList.indexOf(item->text()));
        m_pd->selectedDataList.append(item->text());
    }

    QString text = "";
    for (int var = 0; var < m_pd->selectedDataList.size(); ++var)
    {
        text.append(m_pd->selectedDataList[var]);
        if (var < m_pd->selectedDataList.size() - 1)
        {
            text.append(",");
        }
    }
    lineEdit()->setText(text);
}

#endif // QMultiComboBox_H
