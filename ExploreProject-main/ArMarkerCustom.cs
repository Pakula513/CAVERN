using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ArMarkerCustom : MonoBehaviour
{
    private int index = -1;

    void Awake() {
        ArMarkerCustomManager.reset();
    }

    void Start() {
        index = ArMarkerCustomManager.addNewArMarkerCustom();
    }

    void Update() {
        this.GetComponentInChildren<Renderer>().material.color = ArMarkerCustomManager.getColor(index);
    }
}
