#include "gui/ConfigWidget.h"
#include "globals/Config.h"

#include <QHBoxLayout>
#include <QHeaderView>

// A ready to use config slider widget nested inside a scroll area.
// It's based on a global config object from the framework.
// It dynamically adds sliders to the widget according to what fields
// it finds registered in the config object. Scroll bars are provided
// if needed. Changes of the sliders are mapped to the global config,
// so that they are immediately visible system wide. An interface for
// updating the sliders if the config was changed elsewhere is provided
// in addition to next and previous slider selectors and ticking the
// selected slider up and down (these are mainly for joystick control).
// The scroll area automatically scrolls so that the selected slider
// is visible.

ConfigWidget::ConfigWidget(QWidget *parent)
    : QTreeWidget(parent)
{
//	setStyle(new QCleanlooksStyle());
	setColumnCount(1);
    header()->hide();
//	setAnimated(true);
	setSelectionMode(QTreeWidget::NoSelection);
	setFocusPolicy(Qt::NoFocus);

	connect(this, SIGNAL(itemExpanded(QTreeWidgetItem*)), this, SLOT(scrollOnExpand(QTreeWidgetItem*)));
}

ConfigWidget::~ConfigWidget()
{
	QSettings settings;
	settings.beginGroup("configtree");
	settings.remove(""); // clear group
	settings.endGroup();

	// In the destructor the state of the sliders is saved for the next session.
	saveState(invisibleRootItem());
}

// Prepare the config widget.
// The config widget is filled dynamically with sliders and labels after the config file was loaded.
// Config variables appear in the same order as in the config file.
// It requires config.init() to be called first!
void ConfigWidget::init()
{
	QSettings settings;
	settings.beginGroup("configtree");

	QTreeWidgetItem* root;
	bool parentFound = false;
	int idx = 0;
	foreach (QString configKey, config.memberNames)
	{
		// Reset the root the root of the tree.
		root = invisibleRootItem();

		// Split the config key up on the dot and try to register each part in the tree.
		QStringList configKeyPartList = configKey.split(".");
		QString breadCrumb = "";
		for (int k = 0; k < configKeyPartList.length(); k++)
		{
			QString configKeyPart = configKeyPartList[k];
			if (k > 0)
				breadCrumb += ".";
			breadCrumb += configKeyPart;

			// First let's see if we already have a branch for this part of the key.
			parentFound = false;
			for (int j = 0; j < root->childCount(); j++)
			{
				QTreeWidgetItem* item = root->child(j);
				if (item->text(0) == configKeyPart)
				{
					// Matching tree node found. Recurse.
					root = item;
					parentFound = true;
					break; // next config key part
				}
			}

			// No match found, create a new branch.
			if (!parentFound)
			{
				// Add a new tree widget item to the tree.
				QTreeWidgetItem* newItem = new QTreeWidgetItem();
				newItem->setText(0, configKeyPart);
				newItem->setData(0, Qt::UserRole, idx); // id
				newItem->setData(0, Qt::UserRole+1, breadCrumb); // breadcrumb
				root->addChild(newItem);
				root = newItem;

				if (settings.contains(breadCrumb) && settings.value(breadCrumb).toBool())
					newItem->setExpanded(true);

				// Only leafs of the tree contain sliders.
				if (configKeyPart == configKeyPartList.last())
				{
					// Prepare the slider.
					QSlider* slider = new QSlider(Qt::Horizontal);
					slider->setPageStep(1);
					slider->setMaximum(100);
					slider->setMinimum(-100);
					slider->setValue(config[configKey] / config.sliderFactors[configKey]);
					configSliders << slider;
					connect(slider, SIGNAL(valueChanged(int)), &configSliderMapper, SLOT(map()));
					connect(slider, SIGNAL(sliderPressed()), &configSliderMapper, SLOT(map()));
					configSliderMapper.setMapping(slider, idx);

					// Prepare the config value label.
                    QLabel* valueLabel = new QLabel(QString::number(config[configKey]).left(7));
                    valueLabel->setMinimumWidth(40);
                    valueLabel->setMaximumWidth(40);
					valueLabel->setAlignment(Qt::AlignRight);
					configLabels << valueLabel;

					// Prepare the config name label.
					QLabel* nameLabel = new QLabel(configKeyPart);
					nameLabel->setText(configKeyPart);

					// A little widget contains the config name, slider and value in a horizontal layout.
					QHBoxLayout* layout = new QHBoxLayout();
					layout->setContentsMargins(4, 1, 4, 1);
					layout->addWidget(nameLabel);
					layout->addWidget(slider);
					layout->addWidget(valueLabel);

					QFrame* widget = new QFrame();
					widget->setLayout(layout);

					setItemWidget(newItem, 0, widget);
					treeItems << newItem;
				}
			}
		}

		idx++;
	}

	settings.endGroup();

	connect(&configSliderMapper, SIGNAL(mapped(int)), this, SLOT(configSliderMoved(int)));

	selectedSliderIndex = 0;
	configSliders[selectedSliderIndex]->setFocus();
}

// Handles changes of the config sliders with the mouse.
// This slot is also triggered when ticking the slider with the joystick buttons.
// It updates the value in the label, the config objects, and sends notification to connected objects.
void ConfigWidget::configSliderMoved(int idx)
{
	selectedSliderIndex = idx;
	QString key = config.memberNames[idx];
    configLabels[idx]->setText(QString::number(configSliders[idx]->value() * config.sliderFactors[key]).left(7));
	config[key] = configSliders[idx]->value() * config.sliderFactors[key];

	emit configChangedOut();
}

// When an item is expanded, this slot makes sure that the last child is still visible.
void ConfigWidget::scrollOnExpand(QTreeWidgetItem* it)
{
	scrollToItem(it->child(it->childCount()-1));
}

// Updates all sliders and labels according to a new config.
// This is needed in case the config was changed by means other than the sliders from this widget,
// e.g. config reset or the robot changed and new config set was loaded.
void ConfigWidget::configChangedIn()
{
	configSliderMapper.blockSignals(true);

	int idx = 0;
	foreach (QString key, config.memberNames)
	{
		configSliders[idx]->setValue(config[key] / config.sliderFactors[key]);
        configLabels[idx]->setText(QString::number(configSliders[idx]->value() * config.sliderFactors[key]).left(7));
		idx++;
	}

	configSliderMapper.blockSignals(false);
}

// Selects the next slider, focuses it, expands the parents, and scrolls to it if needed.
void ConfigWidget::selectNextSlider()
{
	selectedSliderIndex = (selectedSliderIndex + 1) % configSliders.size();
	configSliders[selectedSliderIndex]->setFocus();

	QTreeWidgetItem* parent = treeItems[selectedSliderIndex]->parent();
	while (parent != invisibleRootItem() && parent != 0)
	{
		parent->setExpanded(true);
		parent = parent->parent();
	}

	scrollToItem(treeItems[selectedSliderIndex]);
}

// Selects the previous slider, focuses it and scrolls to it if needed.
void ConfigWidget::selectPrevSlider()
{
	selectedSliderIndex = selectedSliderIndex > 0 ? selectedSliderIndex-1 : configSliders.size()-1;
	configSliders[selectedSliderIndex]->setFocus();

	QTreeWidgetItem* parent = treeItems[selectedSliderIndex]->parent();
	while (parent != invisibleRootItem() && parent != 0)
	{
		parent->setExpanded(true);
		parent = parent->parent();
	}

	scrollToItem(treeItems[selectedSliderIndex]);
}

// Increases the value of the selected slider by one single step and updates the config with it.
void ConfigWidget::increaseSelectedSlider()
{
	configSliders[selectedSliderIndex]->triggerAction(QAbstractSlider::SliderSingleStepAdd);
}

// Decreases the value of the selected slider by one single step and updates the config with it.
void ConfigWidget::decreaseSelectedSlider()
{
	configSliders[selectedSliderIndex]->triggerAction(QAbstractSlider::SliderSingleStepSub);
}

// Recursively saves the checkbox states in the QSettings.
void ConfigWidget::saveState(QTreeWidgetItem* it)
{
	QSettings settings;
	settings.beginGroup("configtree");

	for (int i = 0; i < it->childCount(); i++)
	{
		QTreeWidgetItem* child = it->child(i);
		if (child->isExpanded())
			settings.setValue(child->data(0, Qt::UserRole+1).toString(), child->isExpanded());
		saveState(child);
	}

	settings.endGroup();
}
