using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class ArMarkerCustomManager : MonoBehaviour
{
    private static int num_of_ar_marker_customs = 0;
    private static ArrayList chosenIndices = new ArrayList();
    private static System.Random r = new System.Random();

    public static void reset() {
        num_of_ar_marker_customs = 0;
        chosenIndices = new ArrayList();
    }

    public static int addNewArMarkerCustom() {
        num_of_ar_marker_customs++;
        return num_of_ar_marker_customs - 1;
    }

    private static void updateChosen()
    {
        for (int i = 0; i < 2; i++) {
            int chosenIndex = r.Next(0, num_of_ar_marker_customs);
            while (chosenIndices.Contains(chosenIndex)) {
                chosenIndex = r.Next(0, num_of_ar_marker_customs);
            }
            chosenIndices.Add(chosenIndex);
        }
    }

    public static Color getColor(int index) {
        if (chosenIndices.Count == 0) updateChosen();
        if ((int) chosenIndices[0] == index) {
            return Color.green;
        } else if ((int) chosenIndices[1] == index) {
            return Color.red;
        } else {
            return Color.white;
        }
    }
}
