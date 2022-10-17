#ifndef RECONFIGURE_CLIENT_GUI_H
#define RECONFIGURE_CLIENT_GUI_H

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <fstream>

#include <QObject>
#include <QVariant>
#include <QString>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QSlider>
#include <QLineEdit>
#include <QFormLayout>
#include <QScrollArea>

#include <dynamic_reconfigure/client.h>
#include <boost/any.hpp>

#include <yaml-cpp/yaml.h> // For decoding enum edit_method

#include "collapsiblegroupbox.h"

namespace rqt_roi_viewpoint_planner
{

class AbstractReconfigureClient;

class AbstractParam : public QObject
{
  Q_OBJECT

protected:
  AbstractReconfigureClient *client;

public:
  const std::string name;
  const std::string type;
  const uint32_t level;
  const std::string description;
  const std::string edit_method;

  AbstractParam(const std::string name, const std::string type, const uint32_t level, const std::string description, const std::string edit_method,
                AbstractReconfigureClient *client)
    : name(name),
      type(type),
      level(level),
      description(description),
      edit_method(edit_method),
      client(client)
  {}

  virtual ~AbstractParam() {}

  virtual boost::any getDefault() const = 0;
  virtual boost::any getMin() const = 0;
  virtual boost::any getMax() const = 0;
  virtual boost::any getValue() const = 0;
  virtual void setValue(const boost::any &val) = 0;
  virtual void updateGuiValue() = 0;
};

using AbstractParamPtr = std::unique_ptr<AbstractParam>;

class AbstractReconfigureClient : public QObject
{
  Q_OBJECT
protected:
  std::vector<AbstractParamPtr> params;
  std::unordered_map<std::string, AbstractParam*> param_map;

  void changedConfig()
  {
    for (const AbstractParamPtr &param : params)
    {
      param->updateGuiValue();
    }
  }

signals:
  void configChanged();

public:
  virtual ~AbstractReconfigureClient() {}

  virtual void sendConfig(const AbstractParam *changed_param) = 0;
};

template<typename C>
class AbstractConfigParam : public AbstractParam
{
private:
  boost::any def;
  boost::any min;
  boost::any max;

public:
  AbstractConfigParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client)
    : AbstractParam (p->name, p->type, p->level, p->description, p->edit_method, client)
  {
    p->getValue(C::__getDefault__(), def);
    p->getValue(C::__getMin__(), min);
    p->getValue(C::__getMax__(), max);
  }

  virtual ~AbstractConfigParam() {}

  virtual boost::any getDefault() const override
  {
    return def;
  }

  virtual boost::any getMin() const override
  {
    return min;
  }

  virtual boost::any getMax() const override
  {
    return max;
  }
};

template<typename C, typename T>
class Param : public AbstractConfigParam<C>
{
private:
  using ParamDescription = typename C::template ParamDescription<T>;
  T &field;

public:
  Param(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config)
    : AbstractConfigParam<C>(p, client),
      field(config.*(reinterpret_cast<const ParamDescription*>(p.get())->field))
  {}

  virtual ~Param() {}

  virtual boost::any getValue() const override
  {
    return field;
  }

  virtual void setValue(const boost::any &val) override
  {
    field = boost::any_cast<T>(val);
  }
};

template<typename T>
inline QVariant toQVariant(const T &val)
{
  return QVariant(val);
}

template<>
inline QVariant toQVariant(const std::string &val)
{
  return QVariant(QString::fromStdString(val));
}

template<typename T>
inline T fromQVariant(const QVariant &val)
{
  return val.value<T>();
}

template<>
inline std::string fromQVariant(const QVariant &val)
{
  return val.toString().toStdString();
}

template<typename C, typename T>
class EnumParam : public Param<C, T>
{
private:
  QComboBox *comboBox;

  void on_comboBox_activated(int index)
  {
    QVariant val = comboBox->itemData(index);
    this->setValue(fromQVariant<T>(val));
    this->client->sendConfig(this);
  }

public:
  EnumParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, T>(p, client, config), comboBox(new QComboBox())
  {
    YAML::Node enum_description = YAML::Load(p->edit_method);
    for (const YAML::Node &node : enum_description["enum"])
    {
        QString entry_name = QString::fromStdString(node["name"].as<std::string>());
        QVariant userdata = toQVariant<T>(node["value"].as<T>());
        entry_name += QString(" (") + userdata.toString() + QString(")");
        comboBox->addItem(entry_name, userdata);
    }

    QVariant default_val;
    default_val = toQVariant<T>(boost::any_cast<T>(this->getDefault()));
    QObject::connect(comboBox, QOverload<int>::of(&QComboBox::activated), this, &EnumParam<C, T>::on_comboBox_activated);

    int default_index = comboBox->findData(default_val);
    if (default_index >= 0)
        comboBox->setCurrentIndex(default_index);

    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), comboBox);
  }

  virtual void updateGuiValue() override
  {
    QVariant val = toQVariant<T>(boost::any_cast<T>(this->getValue()));
    int index = comboBox->findData(val);
    if (index >= 0)
        comboBox->setCurrentIndex(index);
  }
};

template<typename C>
class BoolParam : public Param<C, bool>
{
private:
  QCheckBox *checkBox;

  void on_checkBox_clicked(bool checked)
  {
    this->setValue(checked);
    this->client->sendConfig(this);
  }

public:
  BoolParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, bool>(p, client, config), checkBox(new QCheckBox())
  {
    checkBox->setChecked(boost::any_cast<bool>(this->getDefault()));
    QObject::connect(checkBox, &QCheckBox::clicked, this, &BoolParam<C>::on_checkBox_clicked);
    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), checkBox);
  }

  virtual void updateGuiValue() override
  {
    checkBox->setChecked(boost::any_cast<bool>(this->getValue()));
  }

};

template<typename C>
class IntParam : public Param<C, int>
{
private:
  QSpinBox *spinBox;
  QSlider *slider;

  void intSlider_setValue(int value)
  {
    int minVal = boost::any_cast<int>(this->getMin());
    int maxVal = boost::any_cast<int>(this->getMax());
    int position = qBound(0, static_cast<int>(static_cast<double>(value - minVal) / static_cast<double>(maxVal - minVal) * 100.0), 100);
    slider->setValue(position);
  }

  void intSpinBox_setPosition(int position)
  {
    int minVal = boost::any_cast<int>(this->getMin());
    int maxVal = boost::any_cast<int>(this->getMax());
    spinBox->setValue(minVal + position * (maxVal - minVal) / 100);
  }

  void intValue_sendConfig()
  {
    this->setValue(spinBox->value());
    this->client->sendConfig(this);
  }

  void on_slider_sliderMoved(int position)
  {
    intSpinBox_setPosition(position);
  }

  void on_slider_sliderReleased()
  {
    intValue_sendConfig();
  }

  void on_spinBox_editingFinished()
  {
    intSlider_setValue(spinBox->value());
    intValue_sendConfig();
  }

public:
  IntParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, int>(p, client, config), spinBox(new QSpinBox()), slider(new QSlider(Qt::Orientation::Horizontal))
  {
    int min = boost::any_cast<int>(this->getMin());
    int max = boost::any_cast<int>(this->getMax());
    int def = boost::any_cast<int>(this->getDefault());
    QHBoxLayout *layout = new QHBoxLayout();
    QLabel *minLabel = new QLabel(QString::number(min));
    QLabel *maxLabel = new QLabel(QString::number(max));
    slider->setMaximum(100);
    intSlider_setValue(def);
    spinBox->setMinimum(min);
    spinBox->setMaximum(max);
    spinBox->setValue(def);
    layout->addWidget(minLabel);
    layout->addWidget(slider);
    layout->addWidget(maxLabel);
    layout->addWidget(spinBox);
    QObject::connect(slider, &QSlider::sliderMoved, this, &IntParam<C>::on_slider_sliderMoved);
    QObject::connect(slider, &QSlider::sliderReleased, this, &IntParam<C>::on_slider_sliderReleased);
    QObject::connect(spinBox, &QSpinBox::editingFinished, this, &IntParam<C>::on_spinBox_editingFinished);
    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), layout);
  }

  virtual void updateGuiValue() override
  {
    intSlider_setValue(boost::any_cast<int>(this->getValue()));
    spinBox->setValue(boost::any_cast<int>(this->getValue()));
  }

};

template <typename C>
class DoubleParam : public Param<C, double>
{
private:
  QDoubleSpinBox *spinBox;
  QSlider *slider;

  void doubleSlider_setValue(double value)
  {
    double minVal = boost::any_cast<double>(this->getMin());
    double maxVal = boost::any_cast<double>(this->getMax());
    int position = qBound(0, static_cast<int>((value - minVal) / (maxVal - minVal) * 100.0), 100);
    slider->setValue(position);
  }

  void doubleSpinBox_setPosition(int position)
  {
    double minVal = boost::any_cast<double>(this->getMin());
    double maxVal = boost::any_cast<double>(this->getMax());
    spinBox->setValue(minVal + static_cast<double>(position) / 100.0 * (maxVal - minVal));
  }

  void doubleValue_sendConfig()
  {
    this->setValue(spinBox->value());
    this->client->sendConfig(this);
  }

  void on_slider_sliderMoved(int position)
  {
    doubleSpinBox_setPosition(position);
  }

  void on_slider_sliderReleased()
  {
    doubleValue_sendConfig();
  }

  void on_spinBox_editingFinished()
  {
    doubleSlider_setValue(spinBox->value());
    doubleValue_sendConfig();
  }

public:
  DoubleParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, double>(p, client, config), spinBox(new QDoubleSpinBox()), slider(new QSlider(Qt::Orientation::Horizontal))
  {
    double min = boost::any_cast<double>(this->getMin());
    double max = boost::any_cast<double>(this->getMax());
    double def = boost::any_cast<double>(this->getDefault());
    QHBoxLayout *layout = new QHBoxLayout();
    QLabel *minLabel = new QLabel(QString::number(min));
    QLabel *maxLabel = new QLabel(QString::number(max));
    slider->setMaximum(100);
    doubleSlider_setValue(def);
    spinBox->setMinimum(min);
    spinBox->setMaximum(max);
    int exponent = QString::number(max-min, 'e', 0).split('e').last().toInt();
    spinBox->setSingleStep(std::pow(10.0, exponent-1));
    spinBox->setValue(def);
    layout->addWidget(minLabel);
    layout->addWidget(slider);
    layout->addWidget(maxLabel);
    layout->addWidget(spinBox);
    QObject::connect(slider, &QSlider::sliderMoved, this, &DoubleParam<C>::on_slider_sliderMoved);
    QObject::connect(slider, &QSlider::sliderReleased, this, &DoubleParam<C>::on_slider_sliderReleased);
    QObject::connect(spinBox, &QDoubleSpinBox::editingFinished, this, &DoubleParam<C>::on_spinBox_editingFinished);
    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), layout);
  }

  virtual void updateGuiValue() override
  {
    doubleSlider_setValue(boost::any_cast<double>(this->getValue()));
    spinBox->setValue(boost::any_cast<double>(this->getValue()));
  }
};

template <typename C>
class StringParam : public Param<C, std::string>
{
private:
  QLineEdit *lineEdit;

  void on_lineEdit_textEdited(const QString &text)
  {
    this->setValue(text.toStdString());
    this->client->sendConfig(this);
  }

public:
  StringParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
    : Param<C, std::string>(p, client, config), lineEdit(new QLineEdit())
  {
    lineEdit->setText(QString::fromStdString(boost::any_cast<std::string>(this->getDefault())));
    QObject::connect(lineEdit, &QLineEdit::textEdited, this, &StringParam<C>::on_lineEdit_textEdited);
    configLayout->addRow(new QLabel(QString::fromStdString(this->name)), lineEdit);
  }

  virtual void updateGuiValue() override
  {
    lineEdit->setText(QString::fromStdString(boost::any_cast<std::string>(this->getValue())));
  }

};

template<typename C>
AbstractParamPtr initializeParam(const typename C::AbstractParamDescriptionConstPtr &p, AbstractReconfigureClient *client, C &config, QFormLayout *configLayout)
{
  bool is_enum = p->edit_method != "";
  if (p->type == "bool")
  {
    if (is_enum)
      return std::make_unique<EnumParam<C, bool>>(p, client, config, configLayout);
    else
      return std::make_unique<BoolParam<C>>(p, client, config, configLayout);
  }
  else if (p->type == "int")
  {
    if (is_enum)
      return std::make_unique<EnumParam<C, int>>(p, client, config, configLayout);
    else
      return std::make_unique<IntParam<C>>(p, client, config, configLayout);
  }
  else if (p->type == "double")
  {
    if (is_enum)
      return std::make_unique<EnumParam<C, double>>(p, client, config, configLayout);
    else
      return std::make_unique<DoubleParam<C>>(p, client, config, configLayout);
  }
  else if (p->type == "str")
  {
    if (is_enum)
      return std::make_unique<EnumParam<C, std::string>>(p, client, config, configLayout);
    else
      return std::make_unique<StringParam<C>>(p, client, config, configLayout);
  }
  else
  {
    ROS_WARN_STREAM("Type " << p->type << " of parameter " << p->name << " not implemented");
    return nullptr;
  }
}

template<typename C>
class ReconfigureClient : public AbstractReconfigureClient
{
private:
  class ParamGroup
  {
  private:
    std::string name;
    int id;
    int parent_id;

    CollapsibleGroupBox *group_box;
    QFormLayout *group_layout;

    bool is_initialized;

    inline static std::vector<ParamGroup*> groups;

  public:
    ParamGroup(const std::string &name, int id, int parent_id, QFormLayout *config_layout=nullptr)
      : name(name), id(id), parent_id(parent_id), is_initialized(false)
    {
      if (id == 0) // default group
      {
        group_box = nullptr; // no group box for default group
        group_layout = config_layout;
        is_initialized = true; // no need to initialize default group
      }
      else
      {
        group_box = new CollapsibleGroupBox(QString::fromStdString(name));
        group_layout = new QFormLayout();
        group_box->setCollapsed(true);
        group_box->setLayout(group_layout);
      }

      if (id >= groups.size())
        groups.resize(id + 1);

      groups[id] = this;
    }

    void initialize()
    {
      if (id != 0 && !is_initialized)
      {
        groups[parent_id]->group_layout->addRow(group_box);
        is_initialized = true;
      }
    }

    bool isInitialized()
    {
      return is_initialized;
    }

    QFormLayout* getLayout()
    {
      return group_layout;
    }
  };

  std::string name;
  C current_config;
  dynamic_reconfigure::Client<C> *config_client;
  QTabWidget *tab_widget;
  int tab_index;
  QLineEdit *status_textbox;

  void configCallback(const C &conf)
  {
    ROS_INFO_STREAM("Config callback called");
    current_config = conf;
    tab_widget->setTabEnabled(tab_index, true);
    tab_widget->setCurrentIndex(tab_index);
    emit configChanged();
  }

public:
  ReconfigureClient(const std::string& name, QTabWidget *tab_widget, QLineEdit *statusTextBox)
    : name(name), tab_widget(tab_widget), status_textbox(statusTextBox)
  {
    QWidget *config_widget = new QWidget;
    QFormLayout *config_layout = new QFormLayout(config_widget);

    std::unordered_map<std::string, ParamGroup*> group_map;

    for (const typename C::AbstractGroupDescriptionConstPtr &group : C::__getGroupDescriptions__())
    {
      ParamGroup *gui_group = new ParamGroup(group->name, group->id, group->parent, config_layout);
      for (const typename C::AbstractParamDescriptionConstPtr &param : group->abstract_parameters)
      {
        group_map[param->name] = gui_group;
      }
    }

    for (const typename C::AbstractParamDescriptionConstPtr &param : C::__getParamDescriptions__())
    {
      auto group_it = group_map.find(param->name);
      if (group_it != group_map.end()) // parameter is in group
      {
        ParamGroup *group = group_it->second;
        if (!group->isInitialized())
        {
          group->initialize();
        }
        params.push_back(initializeParam(param, this, current_config, group->getLayout()));
        param_map[param->name] = params.back().get();
      }
      else // should not happen
      {
        ROS_WARN("Parameter not in any group (not even default); should not happen");
        params.push_back(initializeParam(param, this, current_config, config_layout));
        param_map[param->name] = params.back().get();
      }

    }

    QScrollArea * config_scroll_area = new QScrollArea;
    config_scroll_area->setWidgetResizable(true);
    config_scroll_area->setWidget(config_widget);

    tab_index = tab_widget->addTab(config_scroll_area, QString::fromStdString(name));
    tab_widget->setTabEnabled(tab_index, false);

    config_client = new dynamic_reconfigure::Client<C>(name, boost::bind(&ReconfigureClient::configCallback, this, _1));
    QObject::connect(this, &ReconfigureClient::configChanged, this, &ReconfigureClient::changedConfig);
  }

  virtual void sendConfig(const AbstractParam *changed_param)
  {
    if (!config_client->setConfiguration(current_config))
    {
      setStatus(QString::fromStdString(changed_param->name) + " change failed");
      tab_widget->setTabEnabled(tab_index, false);
    }
    else
    {
      setStatus(QString::fromStdString(changed_param->name) + " change successful");
    }
  }

  void setStatus(const QString &message)
  {
    if (status_textbox)
      status_textbox->setText(message);

    ROS_INFO_STREAM(message.toStdString());
  }

  bool saveConfig(const QString &file_path)
  {
    YAML::Node config;
    for (const AbstractParamPtr &p : params)
    {
      if (p->type == "bool")
      {
        config[p->name] = boost::any_cast<bool>(p->getValue()) ? "true" : "false";
      }
      else if (p->type == "int")
      {
        config[p->name] = boost::any_cast<int>(p->getValue());
      }
      else if (p->type == "double")
      {
        config[p->name] = boost::any_cast<double>(p->getValue());
      }
      else if (p->type == "str")
      {
        config[p->name] = boost::any_cast<std::string>(p->getValue());
      }
      else
      {
        ROS_WARN_STREAM("Type " << p->type << " of parameter " << p->name << " not implemented");
        continue;
      }
    }
    YAML::Node config_root;
    config_root[name] = config;

    std::ofstream file(file_path.toStdString());
    if (!file.is_open())
    {
      setStatus(file_path + " could not be opened");
      return false;
    }

    file << config_root;
    file.close();
    return true;
  }

  bool loadConfig(const QString &file_path)
  {
    YAML::Node config_yaml;
    try
    {
      config_yaml = YAML::LoadFile(file_path.toStdString());
    }
    catch (YAML::ParserException)
    {
      setStatus(file_path + " not a valid YAML file");
      return false;
    }
    catch (YAML::BadFile)
    {
      setStatus(file_path + " could not be loaded");
      return false;
    }

    if (!config_yaml.IsMap())
    {
      setStatus("Configuration should be a YAML map");
      return false;
    }

    if (config_yaml[name]) // config is in namespace
    {
      config_yaml = config_yaml[name];
      if (!config_yaml.IsMap())
      {
        setStatus("Configuration should be a YAML map");
        return false;
      }
    }

    size_t read_params = 0;

    for (YAML::const_iterator it = config_yaml.begin(); it != config_yaml.end(); it++)
    {
      std::string name = it->first.as<std::string>();
      auto param_it = param_map.find(name);
      if (param_it == param_map.end())
      {
        ROS_WARN_STREAM("Invalid paramter " << name << " in config");
        continue;
      }
      AbstractParam *p = param_it->second;
      boost::any value;
      try
      {
        if (p->type == "bool")
        {
          value = it->second.as<bool>();
        }
        else if (p->type == "int")
        {
          value = it->second.as<int>();
        }
        else if (p->type == "double")
        {
          value = it->second.as<double>();
        }
        else if (p->type == "str")
        {
          value = it->second.as<std::string>();
        }
        else
        {
          ROS_WARN_STREAM("Type " << p->type << " of parameter " << p->name << " not implemented");
          continue;
        }
      }
      catch(YAML::Exception e)
      {
        ROS_WARN_STREAM("Error parsing value for parameter " << p->name << ": " << e.msg);
        continue;
      }
      p->setValue(value);
      read_params++;
    }

    if (!config_client->setConfiguration(current_config))
    {
      setStatus(QString::number(read_params) + " parameters read for " + QString::fromStdString(name) + ", but could not be sent");
      tab_widget->setTabEnabled(tab_index, false);
      return false;
    }

    setStatus(QString::number(read_params) + " parameters read for " + QString::fromStdString(name));
    return true;
  }
};

} // namespace rqt_roi_viewpoint_planner

#endif // RECONFIGURE_CLIENT_GUI_H
