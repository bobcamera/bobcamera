import { Component, OnDestroy, OnInit, OnChanges, ChangeDetectionStrategy, Input, ViewChild, ElementRef, AfterViewInit, Renderer2, SimpleChanges } from '@angular/core';
import { Subject, Observable } from 'rxjs';

import { MaskCreationComponent } from '../';
import { Point } from '../../models';

declare function parseSVG(svg:string): {};
declare function drawSVGOnCanvas({}, any, number): void;

@Component({
  selector: 'bob-mask-creation-svg',
  templateUrl: './mask-creation-svg.component.html',
  styleUrls: ['./mask-creation-svg.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class MaskCreationSvgComponent extends MaskCreationComponent implements OnInit, OnDestroy, OnChanges, AfterViewInit  {

  @Input()
  set MaskSvg(value: string) {
    this._maskSvg = value;
    if (this._maskSvg) {
      this._maskShapes = parseSVG(this._maskSvg);
    } else {

    }
  }
  get myProperty(): string {
    return this._maskSvg;
  }

  protected _maskSvg: string;
  protected _oldMaskPolygons: Point[][] = [];
  protected _maskShapes = {};

  constructor(protected renderer: Renderer2) {
    super(renderer);
  }

  override ngOnInit(): void {
    super.ngOnInit();
  }

  override ngAfterViewInit(): void {
    super.ngAfterViewInit();
  }

  override ngOnDestroy(): void {
    super.ngOnDestroy();
  }

  ngOnChanges(changes: SimpleChanges) {
  }

  override clearMask() {
    super.clearMask();
    if (this._maskSvg) {
      this._maskShapes = {};
      this.redrawCanvas();
    }    
  }

  override redrawCanvas() {
    this._maskCanvasCtx.clearRect(0, 0, this.maskCanvas.nativeElement.width, this.maskCanvas.nativeElement.height);
    if (this.EditMode) {
      this._polygons.forEach((polygon, index) => {
        const isCurrentPolygon = index === this._polygons.length - 1;
        this.drawPolygon(polygon, this.EditMode && isCurrentPolygon ? 'rgba(0, 0, 0, 0.1)' : 'rgba(0, 0, 0, 0.6)', this.EditMode && isCurrentPolygon);
        });
    } else {
      if (this.maskCanvas.nativeElement.width > 0 && this.maskCanvas.nativeElement.height > 0) {
        drawSVGOnCanvas(this._maskShapes, this.maskCanvas.nativeElement, 0.8)
      }
    }    
  }

  /*protected override updateCanvasDimensions() {
    //console.log('updateCanvasDimensions');
    this.maskCanvas.nativeElement.width = this.streamContainer.nativeElement.clientWidth;
    this.maskCanvas.nativeElement.height = this.streamContainer.nativeElement.clientHeight;
  }*/  

  parseSVG(): void {
    if (this.MaskSvg) {
      this._maskShapes = parseSVG(this.MaskSvg);
    }
  }

  backupPolygons(): void {
    this._oldMaskPolygons = this._polygons;
  }

  restorePolygons(): void {
    this._polygons = this._oldMaskPolygons;
  }
}
