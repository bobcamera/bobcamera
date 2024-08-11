import { Component, OnDestroy, OnInit, ChangeDetectionStrategy, Input, ViewChild, ElementRef, AfterViewInit, Renderer2 } from '@angular/core';
import { Subject, Observable } from 'rxjs';

import { MaskCreationComponent } from '../';

interface Point {x: number; y: number};

@Component({
  selector: 'bob-mask-creation-svg',
  templateUrl: './mask-creation-svg.component.html',
  styleUrls: ['./mask-creation-svg.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class MaskCreationSvgComponent extends MaskCreationComponent implements OnInit, OnDestroy, AfterViewInit  {

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
}
