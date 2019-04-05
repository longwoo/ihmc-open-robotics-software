package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class PointCloudAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableLidarButton;
   @FXML
   private ToggleButton enableLidar2Button;
   @FXML
   private Slider scanHistorySizeSlider;
   @FXML
   private Slider scan2HistorySizeSlider;
   @FXML
   private ToggleButton enableStereoButton;
   @FXML
   private Spinner<Integer> sizeOfPointCloudSpinner;

   private static final int maximumSizeOfPointCloud = 200000;
   private static final int minimumSizeOfPointCloud = 10000;
   public static final int initialSizeOfPointCloud = 50000;

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
         return new Double(newValue.intValue());
      }
   };

   public PointCloudAnchorPaneController()
   {
   }

   public void bindControls()
   {
      load();
      sizeOfPointCloudSpinner.setValueFactory(createNumberOfPointsValueFactory(initialSizeOfPointCloud));
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanShow, enableLidarButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidar2ScanShow, enableLidar2Button.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidar2ScanSize, scan2HistorySizeSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIStereoVisionShow, enableStereoButton.selectedProperty(), true);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.UIStereoVisionSize, sizeOfPointCloudSpinner.getValueFactory().valueProperty());
   }

   @FXML
   public void clearLidar()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UILidarScanClear, true);
   }

   @FXML
   public void clearLidar2()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UILidar2ScanClear, true);
   }

   @FXML
   public void clearStereo()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UIStereoVisionClear, true);
   }

   @FXML
   public void save()
   {
      saveUIControlProperty(REAModuleAPI.UILidarScanShow, enableLidarButton);
      saveUIControlProperty(REAModuleAPI.UILidar2ScanShow, enableLidar2Button);
      saveUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      saveUIControlProperty(REAModuleAPI.UILidar2ScanSize, scan2HistorySizeSlider);
      saveUIControlProperty(REAModuleAPI.UIStereoVisionShow, enableStereoButton);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UILidarScanShow, enableLidarButton);
      loadUIControlProperty(REAModuleAPI.UILidar2ScanShow, enableLidar2Button);
      loadUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      loadUIControlProperty(REAModuleAPI.UILidar2ScanSize, scan2HistorySizeSlider);
      loadUIControlProperty(REAModuleAPI.UIStereoVisionShow, enableStereoButton);
   }

   private IntegerSpinnerValueFactory createNumberOfPointsValueFactory(int initialValue)
   {
      int min = minimumSizeOfPointCloud;
      int max = maximumSizeOfPointCloud;
      int amountToStepBy = minimumSizeOfPointCloud;
      return new IntegerSpinnerValueFactory(min, max, initialValue, amountToStepBy);
   }
}
