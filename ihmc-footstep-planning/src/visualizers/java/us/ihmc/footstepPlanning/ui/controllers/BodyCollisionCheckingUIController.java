package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.footstepPlanning.ui.components.SettableFootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class BodyCollisionCheckingUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty parametersProperty = new FootstepPlannerParametersProperty(this, "bodyCollisionParametersProperty");

   @FXML
   private ToggleButton enableBodyCollisionChecking;

   @FXML
   private Slider bodyWidth;
   @FXML
   private Slider bodyDepth;
   @FXML
   private Slider bodyHeight;

   @FXML
   private Slider bodyBoxBaseX;
   @FXML
   private Slider bodyBoxBaseY;
   @FXML
   private Slider bodyBoxBaseZ;


   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   
   public void bindControls()
   {
      parametersProperty.bidirectionalBindCheckBodyBoxCollisions(enableBodyCollisionChecking.selectedProperty());

      parametersProperty.bidirectionalBindBodyBoxDepth(bodyDepth.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxHeight(bodyHeight.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxWidth(bodyWidth.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxBaseX(bodyBoxBaseX.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxBaseY(bodyBoxBaseY.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxBaseZ(bodyBoxBaseZ.valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerParametersTopic, parametersProperty, createConverter(), true);
   }


   private PropertyToMessageTypeConverter<FootstepPlannerParameters, SettableFootstepPlannerParameters> createConverter()
   {
      return new PropertyToMessageTypeConverter<FootstepPlannerParameters, SettableFootstepPlannerParameters>()
      {
         @Override
         public FootstepPlannerParameters convert(SettableFootstepPlannerParameters propertyValue)
         {
            return propertyValue;
         }

         @Override
         public SettableFootstepPlannerParameters interpret(FootstepPlannerParameters messageContent)
         {
            return new SettableFootstepPlannerParameters(messageContent);
         }
      };
   }
}
