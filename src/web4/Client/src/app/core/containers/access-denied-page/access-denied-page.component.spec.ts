import { ComponentFixture, TestBed } from '@angular/core/testing';

import { AccessDeniedPageComponent } from './access-denied-page.component';

describe('AccessDeniedPageComponent', () => {
  let component: AccessDeniedPageComponent;
  let fixture: ComponentFixture<AccessDeniedPageComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ AccessDeniedPageComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(AccessDeniedPageComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
