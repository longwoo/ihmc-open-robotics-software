package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;

public class OcTreeBasicsAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;
   @FXML
   private Button clearButton;
   // Main OcTree options
   @FXML
   private Slider depthSlider;
   @FXML
   private ComboBox<DisplayType> displayTypeComboBox;
   @FXML
   private ComboBox<ColoringType> coloringTypeComboBox;
   // Lidar buffer options
   @FXML
   private Slider lidarBufferSizeSlider;
   @FXML
   private ToggleButton enableLidarBufferButton;
   @FXML
   private ToggleButton showLidarBufferButton;
   @FXML
   private ToggleButton showInputLidarScanButton;
   // Lidar 2 buffer options
   @FXML
   private Slider lidar2BufferSizeSlider;
   @FXML
   private ToggleButton enableLidar2BufferButton;
   @FXML
   private ToggleButton showLidar2BufferButton;
   @FXML
   private ToggleButton showInputLidar2ScanButton;
   // Stereo vision buffer options
   @FXML
   private Slider stereoBufferSizeSlider;
   @FXML
   private ToggleButton enableStereoBufferButton;
   @FXML
   private ToggleButton showStereoBufferButton;
   @FXML
   private ToggleButton showInputStereoPointCloudButton;

   private final PropertyToMessageTypeConverter<Integer, Number> numberToIntegerConverter = new PropertyToMessageTypeConverter<Integer, Number>()
   {
      @Override
      public Integer convert(Number propertyValue)
      {
         return propertyValue.intValue();
      }

      @Override
      public Number interpret(Integer newValue)
      {
         return new Double(newValue.doubleValue());
      }
   };

   public OcTreeBasicsAnchorPaneController()
   {
   }

   public void setupControls()
   {
      ObservableList<DisplayType> displayTypeOptions = FXCollections.observableArrayList(DisplayType.values());
      displayTypeComboBox.setItems(displayTypeOptions);
      displayTypeComboBox.setValue(DisplayType.PLANE);
      ObservableList<ColoringType> coloringTypeOptions = FXCollections.observableArrayList(ColoringType.values());
      coloringTypeComboBox.setItems(coloringTypeOptions);
      coloringTypeComboBox.setValue(ColoringType.REGION);
      lidarBufferSizeSlider.setLabelFormatter(StringConverterTools.thousandRounding(true));
      lidar2BufferSizeSlider.setLabelFormatter(StringConverterTools.thousandRounding(true));
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeEnable, enableButton.selectedProperty());

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.LidarBufferEnable, enableLidarBufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.LidarBufferOcTreeCapacity, lidarBufferSizeSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.Lidar2BufferEnable, enableLidar2BufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.Lidar2BufferOcTreeCapacity, lidar2BufferSizeSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferEnable, enableStereoBufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferMessageCapacity, stereoBufferSizeSlider.valueProperty(), numberToIntegerConverter);

      load();
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeDepth, depthSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox.valueProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox.valueProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeShowLidarBuffer, showLidarBufferButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeShowLidar2Buffer, showLidar2BufferButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeShowStereoVisionBuffer, showStereoBufferButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanShow, showInputLidarScanButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidar2ScanShow, showInputLidar2ScanButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIStereoVisionShow, showInputStereoPointCloudButton.selectedProperty(), true);

      showInputLidarScanButton.selectedProperty().addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) -> {
         if (oldValue != newValue && !newValue)
            uiMessager.submitMessageInternal(REAModuleAPI.UILidarScanClear, true);
      });

      showInputLidar2ScanButton.selectedProperty().addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) -> {
         if (oldValue != newValue && !newValue)
            uiMessager.submitMessageInternal(REAModuleAPI.UILidar2ScanClear, true);
      });

      showInputStereoPointCloudButton.selectedProperty().addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) -> {
         if (oldValue != newValue && !newValue)
            uiMessager.submitMessageInternal(REAModuleAPI.UIStereoVisionClear, true);
      });
   }

   @FXML
   public void clear()
   {
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeClear, true);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveMainUpdaterConfiguration);
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveBufferConfiguration);

      saveUIControlProperty(REAModuleAPI.UIOcTreeDepth, depthSlider);
      saveUIControlProperty(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox);
      saveUIControlProperty(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox);
      saveUIControlProperty(REAModuleAPI.UIOcTreeShowLidarBuffer, showLidarBufferButton);
      saveUIControlProperty(REAModuleAPI.UILidarScanShow, showInputLidarScanButton);
      saveUIControlProperty(REAModuleAPI.UIOcTreeShowLidar2Buffer, showLidar2BufferButton);
      saveUIControlProperty(REAModuleAPI.UILidar2ScanShow, showInputLidar2ScanButton);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UIOcTreeDepth, depthSlider);
      loadUIControlProperty(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox);
      loadUIControlProperty(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox);
      loadUIControlProperty(REAModuleAPI.UIOcTreeShowLidarBuffer, showLidarBufferButton);
      loadUIControlProperty(REAModuleAPI.UILidarScanShow, showInputLidarScanButton);
      loadUIControlProperty(REAModuleAPI.UIOcTreeShowLidar2Buffer, showLidar2BufferButton);
      loadUIControlProperty(REAModuleAPI.UILidar2ScanShow, showInputLidar2ScanButton);
   }
}
