package us.ihmc.quadrupedUI.uiControllers;

import javafx.fxml.FXML;
import javafx.stage.DirectoryChooser;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.DataSetIOTools;

import java.io.File;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedMenuUIController
{
//   private final DirectoryChooser directoryChooser = new DirectoryChooser();
   private final File defaultDataFolder = new File(DataSetIOTools.RESOURCES_DIRECTORY + File.separator + DataSetIOTools.DATA_SET_DIRECTORY_PATH);
   private Messager messager;
//   private Window ownerWindow;
   private final AtomicReference<String> folderPath = new AtomicReference<>();

   private Topic<String> exportUnitTestPathTopic;
   private Topic<Boolean> exportUnitTestDataFileTopic;


   public void attachMessager(Messager messager)
   {
      this.messager = messager;


      folderPath.set(defaultDataFolder.getAbsolutePath());
   }

//   public void setMainWindow(Window ownerWindow)
//   {
//      this.ownerWindow = ownerWindow;
//   }

   public void setTopics( Topic<String> exportUnitTestPathTopic, Topic<Boolean> exportUnitTestDataFileTopic)
   {
      this.exportUnitTestPathTopic = exportUnitTestPathTopic;
      this.exportUnitTestDataFileTopic = exportUnitTestDataFileTopic;
   }


   /*
   @FXML
   private void browsePlanarRegionOutputFolder()
   {
      directoryChooser.setInitialDirectory(defaultDataFolder);
      File result = directoryChooser.showDialog(ownerWindow);
      if (result == null)
         return;
      String newPath = result.getAbsolutePath();
      messager.submitMessage(exportUnitTestPathTopic, newPath);
      Platform.runLater(() -> folderPath.set(newPath));
   }
       */


   @FXML
   private void savePlanarRegions()
   {
//      messager.submitMessage(exportUnitTestPathTopic, folderPath.get());
//      messager.submitMessage(exportUnitTestDataFileTopic, true);
   }

}
