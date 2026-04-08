#pragma once

#include <atomic>

#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"

#include <QAbstractButton>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QAbstractSpinBox>
#include <QWidget>
#include <QEvent>
#include <QAbstractSlider>
#include <QLineEdit>
#include <QTextEdit>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QLabel>

#include "utils.hpp"

#define DEBUG_QT_NODES false

namespace bt_common_nodes 
{

/**
 * @brief GenericEventListener is a QObject that can be used to listen to any event on a QWidget and execute 
 * a callback function when the event is triggered.
 * The callback function should have the signature: void callback(QEvent* event)
 */
class GenericEventListener : public QObject
{
public:
    explicit GenericEventListener(std::function<void(QEvent*)> callback)
        : callback_(callback) {}

protected:
    bool eventFilter(QObject* obj, QEvent* event) override
    {
        if (callback_)
        {
            callback_(event);
        }
        return QObject::eventFilter(obj, event); // don't block by default
    }

private:
    std::function<void(QEvent*)> callback_;
};

/**
 * @brief Condition node that monitors the state of a Qt widget object.
 * @details A pointer to the monitored widget (QWidget*) is provided via the blackboard using the key defined in
 * the input port. When the widget is interacted with, the node returns success the next time it is ticked.
 */
class WaitForGuiInputEvent : public BT::ConditionNode
{
  public:
    inline static std::string WIDGET_PORT_KEY = "widget";
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>(WIDGET_PORT_KEY) };
    }

    WaitForGuiInputEvent(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config), ok_(true)
    {
        auto widget_key = getBTInput<std::string>(this, WIDGET_PORT_KEY);
        auto* widget = this->config().blackboard->get<QWidget*>(widget_key);
        
        auto* listener = new GenericEventListener([this](QEvent* event) {
            if (event->type() == QEvent::MouseButtonRelease)
            {
                ok_ = false;
            }
            else if (event->type() == QEvent::KeyRelease)
            {
                ok_ = false;
            }

        });
        widget->installEventFilter(listener);
    }

  protected:


    BT::NodeStatus tick() override
    {
      if (!ok_)
      {
          ok_ = true;
          if (DEBUG_QT_NODES) std::cout << "Button clicked, returning SUCCESS" << std::endl;
          return BT::NodeStatus::SUCCESS;
      }

      return BT::NodeStatus::RUNNING;
    }

    std::atomic_bool ok_;
};


/**
 * @brief Condition node that monitors the state of a Qt widget object.
 * @details A pointer to the monitored widget (QWidget*) is provided via the blackboard using the key defined in
 * the input port. When the widget is interacted with, the node returns success. 
 * Will only poll for widget interaction when the node is actively being ticked.
 */
class WaitForGuiInputEventOnTick : public BT::ConditionNode
{
  public:
    inline static std::string WIDGET_PORT_KEY = "widget";
    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>(WIDGET_PORT_KEY) };
    }

    WaitForGuiInputEventOnTick(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config), ok_(true)
    {
        auto widget_key = getBTInput<std::string>(this, WIDGET_PORT_KEY);
        auto* widget = this->config().blackboard->get<QWidget*>(widget_key);
        
        auto* listener = new GenericEventListener([this](QEvent* event) {
          if (running_){
              if (event->type() == QEvent::MouseButtonRelease)
              {
                  ok_ = false;
              }
              else if (event->type() == QEvent::KeyRelease)
              {
                  ok_ = false;
              }
          }

        });
        widget->installEventFilter(listener);
    }

  protected:


    BT::NodeStatus tick() override
    {
      running_ = true;
      if (!ok_)
      {
          ok_ = true;
          running_ = false;
          if (DEBUG_QT_NODES) std::cout << "Button clicked, returning SUCCESS" << std::endl;
          return BT::NodeStatus::SUCCESS;
      }

      return BT::NodeStatus::RUNNING;
    }

    std::atomic_bool ok_;
    std::atomic_bool running_;
};

/**
 * @brief An Action node that retrieves the current value from a specified Qt widget and outputs it to the blackboard.
 * The widget is identified by a name provided via the blackboard, and the value is output to the blackboard with 
 * the key defined in the output port. The node checks the type of the widget and retrieves the value accordingly.
 * Note: This node assumes that the QWidget pointer is stored in the blackboard with the key provided in the input port.
 * Can and SHOULD be extended to support more widget types as needed.
 */
class GetValueFromGui : public BT::SyncActionNode
{
  public:
    inline static std::string WIDGET_KEY = "widget";
    inline static std::string OUTPUT_KEY = "output";

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>(WIDGET_KEY), BT::OutputPort<BT::Any>(OUTPUT_KEY) };
    }

    GetValueFromGui(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus tick() override
    {
        auto widget_name = getBTInput<std::string>(this, WIDGET_KEY);
        auto* widget = this->config().blackboard->get<QWidget*>(widget_name);

        if (auto* spin_box = qobject_cast<QAbstractSpinBox*>(widget)) {
            if (auto* int_spin = qobject_cast<QSpinBox*>(widget))
            {
                setOutput(OUTPUT_KEY, int_spin->value());
                if (DEBUG_QT_NODES) std::cout << "SpinBox value: " << int_spin->value() << std::endl;
            }
            else if (auto* double_spin = qobject_cast<QDoubleSpinBox*>(widget))
            {
                setOutput(OUTPUT_KEY, double_spin->value());
                if (DEBUG_QT_NODES) std::cout << "DoubleSpinBox value: " << double_spin->value() << std::endl;
            }
        }
        else if (auto* check_box = qobject_cast<QCheckBox*>(widget))
        {
            setOutput(OUTPUT_KEY, check_box->isChecked());
            if (DEBUG_QT_NODES) std::cout << "CheckBox value: " << check_box->isChecked() << std::endl;
        }
        else if (auto* slider = qobject_cast<QAbstractSlider*>(widget))
        {
            setOutput(OUTPUT_KEY, slider->value());
            if (DEBUG_QT_NODES) std::cout << "Slider value: " << slider->value() << std::endl;
        }
        else if (auto* combo_box = qobject_cast<QComboBox*>(widget))
        {
            setOutput(OUTPUT_KEY, combo_box->currentText().toStdString());
            if (DEBUG_QT_NODES) std::cout << "ComboBox value: " << combo_box->currentText().toStdString() << std::endl;
        }
        else if (auto* line_edit = qobject_cast<QLineEdit*>(widget))
        {
            setOutput(OUTPUT_KEY, line_edit->text().toStdString());
            if (DEBUG_QT_NODES) std::cout << "LineEdit value: " << line_edit->text().toStdString() << std::endl;
        }
        else if (auto* text_edit = qobject_cast<QTextEdit*>(widget))
        {
            setOutput(OUTPUT_KEY, text_edit->toPlainText().toStdString());
            if (DEBUG_QT_NODES) std::cout << "TextEdit value: " << text_edit->toPlainText().toStdString() << std::endl;
        }
        else
        {
            std::cerr << "Widget is not a supported type, please use a supported type or extend the GetValueFromGui node to support the type of widget you are using" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * @brief An Action node that sets a specified value to a Qt widget. The widget is identified by a name provided via the blackboard, 
 * and the value is provided via an input port. The node checks the type of the widget and sets 
 * the value accordingly. Note: This node assumes that the QWidget pointer is stored in the blackboard with the 
 * key provided in the input port. 
 * Can and SHOULD be extended to support more widget types as needed.
 */
class SetGuiValue : public BT::SyncActionNode
{
  public:
    inline static std::string WIDGET_KEY = "widget";
    inline static std::string INPUT_KEY = "input";

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::string>(WIDGET_KEY), BT::InputPort<std::string>(INPUT_KEY) };
    }

    SetGuiValue(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
    {}

    BT::NodeStatus tick() override
    {
        auto widget_name = getBTInput<std::string>(this, WIDGET_KEY);
        auto* widget = this->config().blackboard->get<QWidget*>(widget_name);

        auto input_value = getBTInput<std::string>(this, INPUT_KEY);

        if (auto* spin_box = qobject_cast<QAbstractSpinBox*>(widget)) {
            if (auto* int_spin = qobject_cast<QSpinBox*>(widget))
            {
                int_spin->setValue(std::stoi(input_value));
                if (DEBUG_QT_NODES) std::cout << "Set SpinBox value: " << std::stoi(input_value) << std::endl;
            }
            else if (auto* double_spin = qobject_cast<QDoubleSpinBox*>(widget))
            {
                double_spin->setValue(std::stod(input_value));
                if (DEBUG_QT_NODES) std::cout << "Set DoubleSpinBox value: " << std::stod(input_value) << std::endl;
            }
        }
        else if (auto* check_box = qobject_cast<QCheckBox*>(widget))
        {
            bool checked = (input_value == "true" || input_value == "1");
            check_box->setChecked(checked);
            if (DEBUG_QT_NODES) std::cout << "Set CheckBox value: " << checked << std::endl;
        }
        else if (auto* slider = qobject_cast<QAbstractSlider*>(widget))
        {
            slider->setValue(std::stoi(input_value));
            if (DEBUG_QT_NODES) std::cout << "Set Slider value: " << std::stoi(input_value) << std::endl;
        }
        else if (auto* combo_box = qobject_cast<QComboBox*>(widget))
        {
            int index = combo_box->findText(QString::fromStdString(input_value));
            if (index != -1) {
                combo_box->setCurrentIndex(index);
                if (DEBUG_QT_NODES) std::cout << "Set ComboBox value: " << input_value << std::endl;
            } else {
                std::cerr << "Value not found in ComboBox options" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
        }
        else if (auto* line_edit = qobject_cast<QLineEdit*>(widget))
        {
            line_edit->setText(QString::fromStdString(input_value));
            if (DEBUG_QT_NODES) std::cout << "Set LineEdit value: " << input_value << std::endl;
        }
        else if (auto* text_edit = qobject_cast<QTextEdit*>(widget))
        {
            text_edit->setPlainText(QString::fromStdString(input_value));
            if (DEBUG_QT_NODES) std::cout << "Set TextEdit value: " << input_value << std::endl;
        }
        else
        {
            std::cerr << "Widget is not a supported type for setting value, please use a supported type or extend the SetGuiValue node to support the type of widget you are using" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;
    }
};

/**
 * @brief An Action node that opens a file dialog to select a file. 
 * The dialog title, file filter, and start folder can be specified via input ports.
 */
class OpenFileDialog : public BT::SyncActionNode
{
  public:
    inline static std::string DIALOG_TITLE_PORT_KEY = "dialog_title";
    inline static std::string FILE_FILTER_PORT_KEY = "file_filter";
    inline static std::string START_FOLDER_PORT_KEY = "start_folder";
    inline static std::string SELECTED_FILE_PATH_OUTPUT_PORT_KEY = "selected_file_path";

    OpenFileDialog(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config)
    {}
    
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>(DIALOG_TITLE_PORT_KEY, "Select a file"),
                BT::InputPort<std::string>(FILE_FILTER_PORT_KEY, "YAML files (*.yaml);;All files (*.*)"),
                BT::InputPort<std::string>(START_FOLDER_PORT_KEY, "~/"),
                BT::OutputPort<std::string>(SELECTED_FILE_PATH_OUTPUT_PORT_KEY)};
    }

    BT::NodeStatus tick() override
    {
      // open file dialog to select file
      auto main_gui = this->config().blackboard->get<QWidget*>("main_gui");
      if (!main_gui) {
          return BT::NodeStatus::FAILURE;
      }
      QString dialog_title = getBTInput<std::string>(this, DIALOG_TITLE_PORT_KEY).c_str();
      QString file_filter = getBTInput<std::string>(this, FILE_FILTER_PORT_KEY).c_str();
      QString start_folder = getBTInput<std::string>(this, START_FOLDER_PORT_KEY).c_str();

      QString initial_dir;
      if (start_folder != "") {
          // if (std::filesystem::exists(start_folder)) {
              initial_dir = QDir::homePath() + start_folder;
          // }
      }
      else {
          initial_dir = QDir::homePath() + "/";
      }
      QString fileName = QFileDialog::getOpenFileName(main_gui, dialog_title, initial_dir, file_filter);
      if (fileName.isEmpty()) {
          return BT::NodeStatus::FAILURE;
      }

      setOutput(SELECTED_FILE_PATH_OUTPUT_PORT_KEY, fileName.toStdString());
      return BT::NodeStatus::SUCCESS;
    }

};

/**
 * @brief An Action node that opens a directory dialog to select a folder.
 * The dialog title and start folder can be specified via input ports.
 * The selected directory path is output to the blackboard with the key defined in the output port.
 */
class OpenDirectoryDialog : public BT::SyncActionNode
{
    public:
        OpenDirectoryDialog(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
        
        inline static std::string DIALOG_TITLE_PORT_KEY = "dialog_title";
        inline static std::string SELECTED_DIRECTORY_PATH_OUTPUT_PORT_KEY = "selected_directory_path";
        inline static std::string START_FOLDER_PORT_KEY = "start_folder";

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<std::string>(DIALOG_TITLE_PORT_KEY, "Select a directory"),
                    BT::InputPort<std::string>(START_FOLDER_PORT_KEY, "~/"),
                    BT::OutputPort<std::string>(SELECTED_DIRECTORY_PATH_OUTPUT_PORT_KEY)};
        }
            
        BT::NodeStatus tick() override {
            // open directory dialog to select a folder
            auto main_gui = this->config().blackboard->get<QWidget*>("main_gui");
            if (!main_gui) {
                return BT::NodeStatus::FAILURE;
            }
            QString dialog_title = getBTInput<std::string>(this, DIALOG_TITLE_PORT_KEY).c_str();

            QString initial_dir;
            if (start_folder != "") {
                // if (std::filesystem::exists(start_folder)) {
                    initial_dir = QDir::homePath() + start_folder;
                // }
            }
            else {
                initial_dir = QDir::homePath() + "/";
            }
            
            QString dir_path = QFileDialog::getExistingDirectory(main_gui, dialog_title, initial_dir);
            if (dir_path.isEmpty()) {
                return BT::NodeStatus::FAILURE;
            }

            setOutput(SELECTED_DIRECTORY_PATH_OUTPUT_PORT_KEY, dir_path.toStdString());
            return BT::NodeStatus::SUCCESS;
        }

};


} // bt_common_nodes